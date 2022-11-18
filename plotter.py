import matplotlib.pyplot as plt
import sys
import re
ls = sys.stdin.readlines()
ls = [re.match(r'<([^,]+), ([^,]+), [^,>]+> <([^,]+), ([^,]+), [^,>]+>', e)
      for e in ls]


def f(e):
    a, b, c, d = (float(e) for e in e.groups())

    return ((a, b), (c, d))


ls = [f(e) for e in ls if e]

p1, p2 = zip(*ls)

p1 = list(zip(*p1))
p2 = list(zip(*p2))

plt.axis('equal')
plt.scatter(p1[0], p1[1])
plt.scatter(p2[0], p2[1])
plt.title('1.95π')
plt.show()
