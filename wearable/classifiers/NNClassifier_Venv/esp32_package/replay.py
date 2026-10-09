"""Send held-out DATASET samples to the ESP32 and compare PC/board probabilities."""
import argparse
import time
import numpy as np
import serial
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--port',required=True,help='e.g. COM5')
p.add_argument('--count',type=int,default=10)
a=p.parse_args()
data=np.load(Path(__file__).resolve().parent/'output/replay_test.npz')
with serial.Serial(a.port,115200,timeout=.2,write_timeout=5) as s:
    time.sleep(2)
    s.reset_input_buffer(); s.write(b'RESET\n')
    deadline=time.monotonic()+5
    while s.readline().strip()!=b'READY':
        if time.monotonic()>deadline: raise TimeoutError('Board not ready. Close Serial Monitor and verify the sketch/port.')
    for i in range(min(a.count,len(data['windows']))):
        for row in data['windows'][i]:
            s.write(f'{row[0]:.9g},{row[1]:.9g}\n'.encode())
            time.sleep(.002)  # replay pacing; not a sensor sampling timer
        deadline=time.monotonic()+5
        while True:
            line=s.readline().decode(errors='replace').strip()
            if line=='ERROR': raise RuntimeError('Board rejected a sample.')
            if line.startswith('RESULT,'): break
            if time.monotonic()>deadline: raise TimeoutError('No prediction received.')
        fields=line.split(','); board=int(fields[1]); probs=np.array(fields[2:],dtype=float)
        expected=int(data['predictions'][i])
        error=np.max(np.abs(probs-data['probabilities'][i]))
        print(f'Window {i}: true={data["labels"][i]}, PC={expected}, ESP32={board}, probability error={error:.2g}')
        if board!=expected or error>1e-4: raise AssertionError('PC/ESP32 mismatch: check that model_data.h matches this training run.')
print('Replay passed. This verifies deployment parity, not live-sensor accuracy.')
