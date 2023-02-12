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

## Notes - Dec 2
- How long should the collision
- turn off gravity feature
- check if collision velocity goes down during collision - should be highest
- make the big timestep like 1

## Notes - Dec 9
- bug: elastic collision if 10x radius diff

## Notes - Jan 27
- still does many small time steps even if big time step count > desired steps
- test: vary big time step
- test: big time step in middle of collision
- test: 1 big step before collision and one near the end
- so if time steps are really big its well behaved
- too many small steps taken
- adjust small time step counter: only count collision small steps

## Notes - Feb 3
- test analysis: 
    ```radius_0,radius_1,desired_impact_vel,time_step,rho,coeff_of_res,max_pen_depth_percent_0,max_pen_depth_percent_1,collision_steps,real_impact_vel,desired_collision_step_count
    ```

- current method of finding the exit velocity doesn't work if there is only 1 collision step

## Notes - Feb 10
- various epsilons: 0.1 to 1
- find thesis comittee members - mehta and who?

- also: the early_quit method currently requires no_gravity
- TODO: f64's might be negative when I expect them to be positive