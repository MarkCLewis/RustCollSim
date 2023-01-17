import re
import matplotlib.pyplot as plt
re_particle = re.compile(r'<([^,>]+),[^,>]+,[^,>]+>,<([^,>]+),[^,>]+,[^,>]+>')

with open('out.log') as f:
    data = [e[len('[plot]'):].strip().split(';')
            for e in f.readlines() if e.startswith('[plot]')]

time = [e[0] for e in data]

# written for 2 particles
particle0 = [[float(d) for d in re_particle.match(e[1]).groups()]
             for e in data]
particle1 = [[float(d) for d in re_particle.match(e[2]).groups()]
             for e in data]


# print(particle0[0])

plt.plot(time, [e[1] for e in particle0])
plt.show()
