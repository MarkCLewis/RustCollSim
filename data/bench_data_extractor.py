from __future__ import annotations
from pathlib import Path
from dataclasses import dataclass

run_data = {}


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


data_files = {}
for file in Path('data').glob('bench*.txt'):
    print(f'Processing {file}')
    with open(file, 'r') as f:
        lines = f.readlines()

    for line in lines:
        if line.startswith('BENCH'):
            key, value = RunSetup.parse_line(line[len('BENCH'):].strip())

            if key not in data_files:
                data_files[key] = []
            data_files[key].append(value)

for key, values in data_files.items():
    print(key, sum(values) / len(values))
