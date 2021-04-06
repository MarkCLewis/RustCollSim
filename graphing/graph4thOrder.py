from typing import Any, Dict, List, Tuple
import matplotlib.pyplot as plt
import sys
import json
import math

if len(sys.argv) < 2:
    print('Usage: graph4thOrder.py file')

with open(sys.argv[1]) as f:
    text: str = f.read()

text = text.split('INIT\n')[1]
text = text.split('thread \'main\' panicked')[0]

# if not text.endswith('}\n'):
#     text += "]}"

run: Dict[str, Any] = json.loads(text)


dt: float = run['dt']
rho: float = run['rho']
r0: float
r1: float
r0, r1 = run['radii']
mass0, mass1 = [4/3 * math.pi * r * r * r * rho for r in (r0, r1)]

def mag(vec: Tuple[float, float, float]) -> float:
    x, y, z = vec
    return (x**2 + y**2 + z**2) ** 0.5

class TimeMoment:

    def __init__(self, d: Dict[str, Any]):
        self.time: float = d['time']
        states: List[Dict[str, List[float]]] = d['states']
        self.particles_x = [s['displacement'] for s in states]
        self.particles_v = [s['velocity'] for s in states]
        self.KE = [s['KE'] for s in states]
        self.PE = [s['PE'] for s in states]
        self.acceleration = [s['acceleration'] for s in states]

        self.isColliding = self.delta() < 2*1e-7

    def delta(self) -> float:
        assert len(self.particles_x) == 2
        (x1, y1, z1), (x2, y2, z2) = self.particles_x
        return ((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2) ** 0.5

    def deltaV(self) -> float:
        assert len(self.particles_v) == 2
        (x1, y1, z1), (x2, y2, z2) = self.particles_v
        return ((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2) ** 0.5

    def __repr__(self):
        return f'TimeMoment({self.time}, {self.particles})'

data: List[TimeMoment] = list(map(TimeMoment, run['data']))

# print(data)


enter = None
enter_velocities = None
enter_E = None
for i, e in enumerate(data):
    if e.isColliding:
        enter = e.time
        enter_velocities = map(mag, e.particles_v)
        print("collision v0 =", e.deltaV())
        enter_E = [a + b for a, b in zip(e.KE, e.PE)]
        break

exit_time = None
exit_velocities = None
exit_E = None
for e in data[i:]:
    if not e.isColliding:
        exit_velocities = map(mag, e.particles_v)
        exit_time = e.time
        exit_E = [a + b for a, b in zip(e.KE, e.PE)]
        break

minVal = min(map(lambda x: x.delta() - (r0+r1), data))

print(f'enter_time = {enter}, exit_time = {exit_time}')
print(f'Max pen depth = {minVal/1e-7}')

# assert enter and exit_time, 'did not find collision'
print(enter, exit_time)

if enter_velocities and exit_velocities:
    enter_velocities, exit_velocities = list(enter_velocities), list(exit_velocities)
    print(enter_velocities, exit_velocities)

# KE = 1/2 m v^2
# 
# KE/KE = v^2 / v^2 (rest cancels)
# 
if enter_velocities and exit_velocities:
    print('Coeff of Res. =', [a / b for a, b in zip(exit_velocities, enter_velocities)])
# print('Coeff of Res. Try 2 =', [a / b for a, b in zip(exit_E, enter_E)])


t = [e.time for e in data]
d = [e.particles_x[0][0] - e.particles_x[1][0] for e in data]
plt.plot(t, d)
# d = [e.particles_x[1][0] for e in data]
# plt.plot(t, d)
if enter:
    plt.axvline(enter, linestyle='dotted')
if exit_time:
    plt.axvline(exit_time, linestyle='dotted')
plt.title('Position vs time')
plt.show()

d = [e.particles_v[0][0] for e in data]
plt.plot(t, d)
d = [e.particles_v[1][0] for e in data]
plt.plot(t, d)
if enter:
    plt.axvline(enter, linestyle='dotted')
if exit_time:
    plt.axvline(exit_time, linestyle='dotted')
plt.title('Velocity vs time')
plt.show()

print(len(d))
t = [e.time for e in data][1:]
d = [e.acceleration[0][0] for e in data][1:]
plt.plot(t, d)
d = [e.acceleration[1][0] for e in data][1:]
plt.plot(t, d)
if enter:
    plt.axvline(enter, linestyle='dotted')
if exit_time:
    plt.axvline(exit_time, linestyle='dotted')
plt.title('Acceleration vs time')
plt.show()

# d = [e.KE[0] + e.PE[0] for e in data]
# plt.plot(t, d)
# d = [e.KE[1] + e.PE[1] for e in data]
# plt.plot(t, d)
# plt.axvline(enter, linestyle='dotted')
# plt.axvline(exit_time, linestyle='dotted')
# plt.show()
