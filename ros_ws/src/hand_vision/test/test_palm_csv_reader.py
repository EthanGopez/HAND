"""Check palm CSV following and row rejection without a camera or a robot."""

import os

import pytest

from hand_vision.palm_csv_reader import EXPECTED_HEADER, PalmCsvReader

HEADER = ','.join(EXPECTED_HEADER) + '\r\n'


@pytest.fixture
def csv_file(tmp_path):
    return tmp_path / 'palm_center.csv'


def write(path, text, mode='a'):
    with open(path, mode, newline='') as handle:
        handle.write(text)


def row(timestamp_ms, x=0.0, y=0.0, z=4.0):
    return f'{timestamp_ms},{x:.4f},{y:.4f},{z:.4f}\r\n'


def test_missing_file_is_not_an_error(csv_file):
    reader = PalmCsvReader(csv_file)
    assert reader.read() == []
    assert 'Cannot open' in reader.status


def test_follows_rows_appended_after_opening(csv_file):
    write(csv_file, HEADER, 'w')
    reader = PalmCsvReader(csv_file)
    assert reader.read() == []
    write(csv_file, row(100, 0.01) + row(133, 0.02))
    samples = reader.read()
    assert [sample.timestamp_ms for sample in samples] == [100, 133]
    assert samples[1].x == pytest.approx(0.02)
    assert reader.read() == []


def test_existing_rows_are_not_replayed_as_new_motion(csv_file):
    write(csv_file, HEADER + row(10) + row(20), 'w')
    reader = PalmCsvReader(csv_file)
    assert reader.read() == []
    write(csv_file, row(30))
    assert [sample.timestamp_ms for sample in reader.read()] == [30]


def test_replay_mode_returns_the_backlog(csv_file):
    write(csv_file, HEADER + row(10) + row(20), 'w')
    reader = PalmCsvReader(csv_file, from_start=True)
    assert [sample.timestamp_ms for sample in reader.read()] == [10, 20]


def test_half_written_row_is_withheld_until_complete(csv_file):
    write(csv_file, HEADER, 'w')
    reader = PalmCsvReader(csv_file)
    reader.read()
    write(csv_file, '150,0.01')
    assert reader.read() == []
    write(csv_file, '00,0.0200,4.0000\r\n')
    samples = reader.read()
    assert len(samples) == 1
    assert samples[0].x == pytest.approx(0.0100)


def test_restarted_demo_is_picked_up_after_truncation(csv_file):
    write(csv_file, HEADER + row(900), 'w')
    reader = PalmCsvReader(csv_file)
    reader.read()
    # hand_tracking_demo.py opens its CSV with mode 'w' on every run, and its
    # timestamps restart lower than the previous session's.
    write(csv_file, HEADER + row(5), 'w')
    assert [sample.timestamp_ms for sample in reader.read()] == [5]


def test_replaced_file_is_reopened(csv_file):
    write(csv_file, HEADER + row(10), 'w')
    reader = PalmCsvReader(csv_file)
    reader.read()
    os.remove(csv_file)
    write(csv_file, HEADER + row(11) + row(12), 'w')
    assert [sample.timestamp_ms for sample in reader.read()] == [11, 12]


def test_wrong_header_is_refused(csv_file):
    write(csv_file, 'frame_index,timestamp_ms,tracked\r\n1,2,True\r\n', 'w')
    reader = PalmCsvReader(csv_file)
    assert reader.read() == []
    assert 'header' in reader.status


@pytest.mark.parametrize('bad', [
    '100,nan,0.0,4.0\r\n',        # not a number
    '100,inf,0.0,4.0\r\n',        # not finite
    '100,0.0,0.0\r\n',            # short row
    '100,0.0,0.0,4.0,9\r\n',      # long row
    '100,oops,0.0,4.0\r\n',       # unparsable
    '100,0.0,0.0,0.0\r\n',        # depth is never zero
    '100,0.0,0.0,-4.0\r\n',       # depth is never negative
    '100,999.0,0.0,4.0\r\n',      # implausible offset
])
def test_untrustworthy_rows_are_dropped(csv_file, bad):
    write(csv_file, HEADER, 'w')
    reader = PalmCsvReader(csv_file)
    reader.read()
    write(csv_file, bad + row(200, 0.01))
    samples = reader.read()
    assert [sample.timestamp_ms for sample in samples] == [200]
    assert reader.rejected == 1


def test_repeated_and_backwards_timestamps_are_dropped(csv_file):
    write(csv_file, HEADER, 'w')
    reader = PalmCsvReader(csv_file)
    reader.read()
    write(csv_file, row(300) + row(300) + row(250) + row(301))
    assert [sample.timestamp_ms for sample in reader.read()] == [300, 301]
    assert reader.rejected == 2


def test_backlog_is_capped_to_the_newest_rows(csv_file):
    write(csv_file, HEADER, 'w')
    reader = PalmCsvReader(csv_file, max_rows_per_read=5)
    reader.read()
    write(csv_file, ''.join(row(index) for index in range(1, 21)))
    samples = reader.read()
    assert [sample.timestamp_ms for sample in samples] == [16, 17, 18, 19, 20]


def test_real_demo_output_parses(csv_file):
    """The reader must accept a file the tracking demo actually produced."""
    recorded = os.path.join(os.path.dirname(__file__),
                            '..', '..', '..', '..', 'palm_center.csv')
    if not os.path.isfile(recorded):
        pytest.skip('no recorded palm_center.csv in the workspace')
    reader = PalmCsvReader(recorded, from_start=True, max_rows_per_read=100000)
    samples = reader.read()
    if len(samples) <= 100:
        pytest.skip('palm_center.csv has no usable recording (the demo truncates it)')
    assert all(sample.z > 0 for sample in samples)
    # A frame that detected two hands writes two rows under one timestamp, so
    # a few rejections are expected; a systematic parsing failure is not.
    assert reader.rejected < len(samples) / 10
