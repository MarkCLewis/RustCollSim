import matplotlib.pyplot as plt
import sys
import re
ls = sys.stdin.readlines()
ls = [re.match(r'<([^,]+), [^,]+, [^,>]+> <([^,]+), [^,]+, [^,>]+>', e)
      for e in ls]


def f(e):
    a, b = (float(e) for e in e.groups())
    return a, b


ls = [f(e) for e in ls if e]

p1, p2 = zip(*ls)

print(p2[:10])

n = list(range(len(p1)))
plt.scatter(n, p1)
plt.scatter(n[::10], [e + (1e-7) for e in p1[::10]], marker='x')
plt.scatter(n, p2)
plt.scatter(n[::10], [e - (1e-7) for e in p2[::10]], marker='x')
plt.show()
# plt.axis('equal')
# plt.scatter(p1[0], p1[1])
# plt.scatter(p2[0], p2[1])
# plt.title('1.95π')
# plt.show()
