# Collisional Simulations in Rust

This code simulates a soft-sphere collision in a large paramter-space to find the optimal settings to keep collisions under control.

## How to run

`cargo run test`

This will generate a csv file in the data folder with all parameters and the outcome of the collision. The program records the coefficient of restitution, the maximum overlap, the impact velocity and the number of simulation steps that the collision took. 
