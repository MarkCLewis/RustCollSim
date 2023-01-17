from pathlib import Path
from dataclasses import dataclass


@dataclass
class Particle:
    idx: int
    x: float
    y: float
    z: float
    r: float = None

    def from_string(s: str):
        idx, pos, r = s.split(';')
        x, y, z = map(float, pos[1:-1].split(','))
        return Particle(int(idx), x, y, z, float(r))

    def from_string_no_r(s: str):
        idx, pos = s.split(';')
        x, y, z = map(float, pos[1:-1].split(','))
        return Particle(int(idx), x, y, z)

    def distance_from(self, other: 'Particle'):
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5


@dataclass
class StepState:
    particles: list[Particle]

    def from_string(s: str):
        return StepState([Particle.from_string(e) for e in s.split('|') if e.strip()])


@dataclass
class Collision:
    t: float
    p1: Particle
    p2: Particle

    def from_string(s: str):
        _, p1, p2, t = s.split('|')
        assert t.startswith('T')
        t = t[1:]
        return Collision(float(t), Particle.from_string_no_r(p1), Particle.from_string_no_r(p2))


def read():
    with Path('pair_collision_run.txt').open() as f:
        for line in f:
            if line.startswith('S'):
                yield StepState.from_string(line[1:])
            elif line.startswith('C'):
                yield Collision.from_string(line[1:])


def filter_overlapping(events: list[StepState | Collision]):
    for event in events:
        if isinstance(event, StepState):
            if event.particles[0].r + event.particles[1].r > event.particles[0].distance_from(event.particles[1]):
                yield event
        else:
            yield event


def count(it: list):
    '''
    for generators
    '''
    i = 0
    for _ in it:
        i += 1
    return i


if __name__ == '__main__':
    print(count(filter_overlapping(read())))
