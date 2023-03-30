use std::collections::HashMap;

use crate::particle::{Particle, ParticleIndex};

pub struct GridCell {
    pub nodes: Vec<ParticleIndex>,
}

/// Benchmarking suggests this is roughly O(n)
pub struct Grid {
    // cells are square
    cells: HashMap<(i32, i32), GridCell>,
    cell_size: f64,
}

impl Grid {
    pub fn new(largest_particle_radius: f64) -> Self {
        Self {
            cells: HashMap::new(),
            cell_size: largest_particle_radius * 2.5,
        }
    }

    fn push(&mut self, particle: ParticleIndex, x: f64, y: f64, r: f64) {
        if r >= self.cell_size / 2. {
            panic!("particle radius too large for grid");
        }

        let cell_x = (x / self.cell_size).floor() as i32;
        let cell_y = (y / self.cell_size).floor() as i32;
        let grid_cell = self
            .cells
            .entry((cell_x, cell_y))
            .or_insert(GridCell { nodes: Vec::new() });
        grid_cell.nodes.push(particle);
    }

    fn get_neighbors(&self, x: f64, y: f64) -> Vec<ParticleIndex> {
        let cell_x = (x / self.cell_size).floor() as i32;
        let cell_y = (y / self.cell_size).floor() as i32;
        let mut neighbors = Vec::new();
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(grid_cell) = self.cells.get(&(cell_x + dx, cell_y + dy)) {
                    neighbors.extend(grid_cell.nodes.iter().copied());
                }
            }
        }
        neighbors
    }

    fn collides_with(&self, p: Particle, population: &Vec<Particle>) -> bool {
        self.get_neighbors(p.p.x(), p.p.y())
            .into_iter()
            .any(|n| population[n.0].is_colliding(&p))
    }

    fn try_push(
        &mut self,
        particle: Particle,
        particle_idx: ParticleIndex,
        population: &Vec<Particle>,
    ) -> Result<(), ()> {
        if self.collides_with(particle, population) {
            Err(())
        } else {
            self.push(particle_idx, particle.p.x(), particle.p.y(), particle.r);
            Ok(())
        }
    }
}

pub fn generate<F>(count: usize, mut particle_gen: F, largest_particle_radius: f64) -> Vec<Particle>
where
    F: FnMut() -> Particle,
{
    let mut grid = Grid::new(largest_particle_radius);

    let mut population = Vec::new();
    for i in 0..count {
        loop {
            let p = particle_gen();
            if grid.try_push(p, ParticleIndex(i), &population).is_ok() {
                population.push(p);
                break;
            }
        }
    }

    population
}
