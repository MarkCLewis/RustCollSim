# Collisional Simulations in Rust

This repository holds a version of my simulation code written in Rust since I'm tired of trying to maintain old C++ code.

## Outline

- System (global config)
- Population (particle data)
- ForcePQ (sub-steps)
- KDTree (gravity forces)

```mermaid
flowchart TD;
    System-- stores -->Population;
    Population -- remakes --> KDTree;
    KDTree -- applies forces --> Population;
    KDTree -- adds events to pq --> ForcePQ;
    ForcePQ -- applies sub steps --> Population;
```

## One Step

```mermaid
flowchart TD;
    b[Build KDTree] --> c
    subgraph c[Walk KDTree]
    p2[Particle-Particle] --> cdv
    pn[Particle-Node] --> cdv
    cdv[Compute &Delta;v]
    coll[Put possible collision on pq]
    p2 --> coll
    end
    c --> pq[Process PQ]
    pq --> fs[Finalize step]
```


## Notes

- Particle-Node gravity is applied for full step.
- Particle-Particle gravity is applied till next pq event.
- KDTree: $\Delta v$ will be collected and applied at the end of going through all particles.