"""Train a two-channel RMS neural network from EMG_Input.txt.

Input: one label and AT LEAST 2 comma-separated channel values per line, e.g.
    Power_Grip 0.32,20.72,0.28,0.22

Run: python train.py
Optional: python train.py EMG_Input.txt --channels 1 3 --epochs 500
Channel arguments are ONE-BASED; default uses the first two columns.
"""
import argparse
import copy
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neural_network import MLPClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import accuracy_score, balanced_accuracy_score, classification_report, confusion_matrix, log_loss

NAMES = ['rest', 'power_grip', 'two_finger', 'three_finger', 'pointing_index', 'open_hand']
LABEL_MAP = {}
for i in range(len(NAMES)):
    LABEL_MAP[NAMES[i]] = i
INPUT_FILE = 'EMG_Input.txt'  # Put this file beside train.py.
CHANNELS = (1, 2)            # One-based column numbers; choose TWO of 1..4.
WINDOW = 100  # 200 ms at 500 Hz per channel
HOP = 25      # 50 ms



def features(window, mode):
    x = np.asarray(window, dtype=np.float64)
    if mode == 'raw': x = x - x.mean(axis=0, keepdims=True)
    return np.sqrt(np.mean(x*x, axis=0))

def load_trials(path, channels=CHANNELS):
    """Parse the TXT and preserve contiguous segment boundaries."""
    if len(channels) != 2 or len(set(channels)) != 2 or any(c not in range(1, len(NAMES)) for c in channels):
        raise ValueError('Choose two different channel numbers from 1..4.')
    columns = [c - 1 for c in channels]
    trials = {}
    active = None
    previous_label = None
    segment_number = 0
    ignored = 0
    counts = {name: 0 for name in NAMES}

    with Path(path).open('r', encoding='utf-8-sig') as file:
        for line_number, line in enumerate(file, 1):
            if not line.strip():
                # A separator also breaks continuity.
                active = None
                previous_label = None
                continue
            fields = line.split(maxsplit=1)  # Accept spaces or tabs.
            if len(fields) != 2:
                raise ValueError(f'Line {line_number}: expected LABEL followed by four values.')
            name, values = fields
            name = name.casefold()
            try:
                samples = [float(v) for v in values.split(',')]
            except ValueError as exc:
                raise ValueError(f'Line {line_number}: invalid numeric values.') from exc
            if len(samples) != 4:
                raise ValueError(f'Line {line_number}: expected four channels, got {len(samples)}.')
            if not np.isfinite(samples).all():
                raise ValueError(f'Line {line_number}: non-finite sample.')

            if name == 'error':
                ignored += 1
                active = None
                previous_label = None
                continue
            if name not in LABEL_MAP:
                raise ValueError(f'Line {line_number}: unknown label {fields[0]!r}. Check LABEL_MAP.')
            label = LABEL_MAP[name]
            if active is None or name != previous_label:
                segment_number += 1
                key = f'segment_{segment_number:04d}'
                active = {'label': label, 'samples': [], 'start_line': line_number}
                trials[key] = active
            active['samples'].append([samples[c] for c in columns])
            active['end_line'] = line_number
            previous_label = name
            counts[NAMES[label]] += 1

    if not trials:
        raise ValueError('No usable labeled data.')
    for trial in trials.values():
        trial['samples'] = np.asarray(trial['samples'], dtype=np.float64)
    print('Selected channels (one-based):', tuple(channels))
    print('Skipped Error rows:', ignored)
    for label, name in enumerate(NAMES):
        segments = [t for t in trials.values() if t['label'] == label]
        usable = sum(len(t['samples']) >= WINDOW for t in segments)
        print(f'{label}: {name}: {counts[name]} samples, {len(segments)} segments, {usable} usable')
    print('NOTICE: segment IDs are inferred from label changes, not verified repetition IDs.')
    segment_counts = [sum(t['label'] == label for t in trials.values()) for label in range(len(NAMES))]
    if segment_counts[1] > 2 * max(segment_counts[0], *segment_counts[2:]):
        print('Audit Power_Grip labels: there are substantially more segments than other classes.')
    return trials

def partition(trials):
    rng = np.random.default_rng(42)
    parts = [[], [], []]
    for label in range(len(NAMES)):
        keys = [k for k,t in trials.items() if t['label'] == label and len(t['samples']) >= WINDOW]
        rng.shuffle(keys)
        nval = max(1, int(round(.2*len(keys))))
        ntest = nval
        ntrain = len(keys)-nval-ntest
        for dest, ids in zip(parts, [keys[:ntrain], keys[ntrain:ntrain+nval], keys[ntrain+nval:]]): dest.extend(ids)
    return parts

def window_trials(trials, keys, mode):
    x, y, windows, owners = [], [], [], []
    for key in keys:
        t = trials[key]
        for start in range(0, len(t['samples'])-WINDOW+1, HOP):
            w = t['samples'][start:start+WINDOW]
            x.append(features(w, mode)); y.append(t['label']); windows.append(w); owners.append(key)
    return np.asarray(x), np.asarray(y), np.asarray(windows), np.asarray(owners)

def literal(value):
    if np.ndim(value) == 0: return f'{float(value):.9e}f'
    return '{' + ', '.join(literal(v) for v in value) + '}'

def export(model, scaler, out, mode):
    lines = ['#pragma once', '// Generated by train.py. Do not edit learned arrays.',
             'static constexpr int INPUTS=2, HIDDEN=16, CLASSES='+str(len(NAMES))+', WINDOW=100;',
             f'static constexpr bool CENTER_RAW = {str(mode == "raw").lower()};',
             'static const char* const CLASS_NAMES['+str(len(NAMES))+'] = {' + ', '.join(json.dumps(n) for n in NAMES) + '};']
    arrays = [('MEAN', scaler.mean_), ('SCALE', scaler.scale_),
              ('W1', model.coefs_[0]), ('B1', model.intercepts_[0]),
              ('W2', model.coefs_[1]), ('B2', model.intercepts_[1])]
    for name, arr in arrays:
        shape = ''.join(f'[{n}]' for n in arr.shape)
        lines.append(f'static const float {name}{shape} = {literal(arr)};')
    out.write_text('\n'.join(lines)+'\n')

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('input_file', nargs='?', help='TXT path; defaults to EMG_Input.txt beside this script')
    p.add_argument('--channels', type=int, nargs=2, default=CHANNELS,
                   metavar=('CH1', 'CH2'), help='Two different one-based column numbers from 1..4')
    p.add_argument('--mode', choices=['envelope', 'raw'], default='envelope')
    p.add_argument('--epochs', type=int, default=500)
    a = p.parse_args()
    if a.epochs < 1:
        p.error('--epochs must be at least 1')
    if len(set(a.channels)) != 2 or any(c not in range(1, len(NAMES)) for c in a.channels):
        p.error('--channels must contain two different numbers from 1..4')
    root = Path(__file__).resolve().parent
    input_path = Path(a.input_file) if a.input_file else root/INPUT_FILE
    if not input_path.is_file():
        p.error(f'Input not found: {input_path}. Put EMG_Input.txt beside train.py or provide its path.')
    output = root/'output'; output.mkdir(exist_ok=True)
    trials = load_trials(input_path, a.channels)
    ids = partition(trials)
    splits = [window_trials(trials, k, a.mode) for k in ids]
    xtrain, ytrain = splits[0][:2]; xval, yval = splits[1][:2]; xtest, ytest = splits[2][:2]
    scaler = StandardScaler().fit(xtrain)  # TRAIN ONLY
    train = scaler.transform(xtrain); val = scaler.transform(xval); test = scaler.transform(xtest)
    model = MLPClassifier(hidden_layer_sizes=(16,), activation='relu', solver='adam',
                          alpha=.001, batch_size=min(64,len(train)), learning_rate_init=.001, random_state=42)
    best, best_loss, stale, history = None, float('inf'), 0, []
    for epoch in range(a.epochs):
        model.partial_fit(train, ytrain, classes=np.arange(len(NAMES)))
        loss = log_loss(yval, model.predict_proba(val), labels=np.arange(len(NAMES)))
        history.append([model.loss_, loss])
        if loss < best_loss-1e-5:
            best, best_loss, stale = copy.deepcopy(model), loss, 0
        else: stale += 1
        if epoch % 25 == 0: print(f'Epoch {epoch+1}: train loss={model.loss_:.4f}, validation loss={loss:.4f}')
        if stale >= 40: break
    model = best
    pred = model.predict(test)
    print('Window test accuracy:', accuracy_score(ytest,pred))
    print('Balanced window test accuracy:', balanced_accuracy_score(ytest,pred))
    print(classification_report(ytest,pred,labels=np.arange(len(NAMES)),target_names=NAMES,zero_division=0))
    print('Confusion matrix, rows=true / columns=predicted:\n',confusion_matrix(ytest,pred,labels=np.arange(len(NAMES))))
    # Trial-majority accuracy makes the evaluation unit visible.
    owners = splits[2][3]
    trial_pred = [np.bincount(pred[owners==k],minlength=len(NAMES)).argmax() for k in ids[2]]
    trial_true = [trials[k]['label'] for k in ids[2]]
    print('Held-out whole-trial majority accuracy:',accuracy_score(trial_true,trial_pred))
    header = root/'firmware/grip_classifier/model_data.h'
    header.parent.mkdir(parents=True, exist_ok=True)
    export(model, scaler, header, a.mode)
    np.savez(output/'replay_test.npz', windows=splits[2][2], labels=ytest, predictions=pred,
             probabilities=model.predict_proba(test), mode=a.mode, channels=np.asarray(a.channels))
    (output/'split.json').write_text(json.dumps(dict(zip(['train','validation','test'],ids)),indent=2))
    metadata = {
        'input_file': str(input_path.resolve()), 'channels_one_based': list(a.channels),
        'mode': a.mode, 'sampling_rate_assumed_hz': 500, 'window_samples': WINDOW,
        'hop_samples': HOP, 'class_names': NAMES,
        'trials': {k: {'label': t['label'], 'start_line': t['start_line'],
                       'end_line': t['end_line'], 'sample_count': len(t['samples'])}
                   for k, t in trials.items()},
    }
    (output/'metadata.json').write_text(json.dumps(metadata, indent=2))
    plt.figure(figsize=(6,4))
    for label in range(len(NAMES)):
        keep = ytrain==label
        plt.scatter(xtrain[keep,0],xtrain[keep,1],s=len(NAMES),alpha=.4,label=NAMES[label])
    plt.xlabel('Channel 1 RMS (recording units)'); plt.ylabel('Channel 2 RMS (recording units)')
    plt.legend(fontsize=8); plt.tight_layout(); plt.savefig(output/'features.png',dpi=150); plt.close()
    plt.plot(np.asarray(history)); plt.legend(['train loss','validation loss'])
    plt.xlabel('Epoch'); plt.ylabel('Cross-entropy'); plt.tight_layout(); plt.savefig(output/'loss.png',dpi=150); plt.close()
    print('Exported model_data.h and output/replay_test.npz. Accuracy describes held-out label segments in this file only.')

if __name__ == '__main__': main()
