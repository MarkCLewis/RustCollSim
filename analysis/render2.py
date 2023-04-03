import matplotlib.pyplot as plt
particles = {}

with open('run.log') as f:
    lines = [e[1:] for e in f.read().split('\n') if e.startswith('P')]

for line in lines:
    particle, x, y = line.split()
    particle = int(particle)
    x = float(x)
    y = float(y)

    if particle not in particles:
        particles[particle] = []

    particles[particle].append((x, y))

fig, ax = plt.subplots()
ax.set_aspect('equal')

for particle, points in particles.items():
    ax.scatter(*zip(*points),
               label=f'Particle {particle}')
    for x, y in points:
        ax.add_patch(plt.Circle((x, y), 1e-7, color='black',
                     fill=False, linewidth=0.1))
plt.legend()
plt.show()
