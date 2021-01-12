
# Blending Forces Smoothly

- f(x) = -kx + p
- g(x) = -m<sub>1</sub> m<sub>2</sub> / (x - 10)<sup>2</sup>
- s(x) = e<sup>x</sup> / (e<sup>x</sup> + 1)
  
- F(x) = f(x) * s(xb)
- G(x) = g(x) * s(-xb)

- y = F(x) + G(x)
- x < 0.2

x is distance between surfaces, x < 0 if there is a collision  
y is force

 - how many derivs do we need?
 - simpsons rule - quadratic