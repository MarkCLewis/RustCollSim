from __future__ import annotations
from pathlib import Path
from dataclasses import dataclass


@dataclass(frozen=True, eq=True)
class RunSetup:
    ring: str
    pq: bool
    size: int

    @classmethod
    def parse_line(cls, line: str) -> tuple[RunSetup, float]:
        data = dict([[value.strip() for value in e.split('=', maxsplit=1)]
                     for e in line.split(',')])

        assert data['ring'] in ['A', 'B']
        assert data['pq'] in ['true', 'false']
        return cls(data['ring'], data['pq'] == 'true', int(data['size'])), float(data['runtime'])


def load_data():
    data_files = {}
    for file in Path('.').glob('bench*.txt'):
        print(f'Processing {file}')
        with open(file, 'r') as f:
            lines = f.readlines()

        for line in lines:
            if line.startswith('BENCH'):
                key, value = RunSetup.parse_line(line[len('BENCH'):].strip())

                if key not in data_files:
                    data_files[key] = []
                data_files[key].append(value)

    assert len(data_files) > 0, 'no data got loaded'
    return data_files


def averages(data_files):
    return {key: sum(values) / len(values) for key, values in data_files.items()}


if __name__ == '__main__':
    data_files = load_data()

    data_average = averages(data_files)

    for key, value in data_average.items():
        print(key, value)

    scaling = []
    for s in (1_000, 10_000, 100_000):
        for ring in ('A', 'B'):
            key_pq = RunSetup(ring, True, s)
            key_npq = RunSetup(ring, False, s)

            value_pq = data_average[key_pq]
            value_npq = data_average[key_npq]

            print(
                f'{s} {ring} Scaling: {value_npq / value_pq:.2f}x')
            scaling.append(value_npq / value_pq)

    print('Average scaling:', sum(scaling) / len(scaling))
