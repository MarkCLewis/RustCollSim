use crate::particle::Particle;

pub fn check_deep_overlap(particle: &[Particle]) {
    for (i, p1) in particle.iter().enumerate() {
        for p2 in particle[i + 1..].iter() {
            if let Some(frac) = p1.overlap_fraction(p2) {
                if frac > 0.2 {
                    eprintln!(
                        "Overlap of {:.2}% between particles {} and {}",
                        frac * 100.,
                        i,
                        i + 1
                    )
                }
            }
        }
    }
}
