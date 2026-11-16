//! Noise that repeats from a seed: the same seed gives the same sensor streams.
//!
//! SplitMix64 for the numbers and Box-Muller for the bell curve: small, fast and with no
//! dependency, which is all a simulated sensor needs (not cryptography).

/// A seeded source of random numbers.
#[derive(Debug, Clone)]
pub struct Noise {
    state: u64,
    spare: Option<f64>,
}

impl Noise {
    #[must_use]
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed,
            spare: None,
        }
    }

    /// Another stream, its own seed taken from this one: one per sensor, so adding a
    /// sensor doesn't change another's noise.
    #[must_use]
    pub fn fork(&mut self) -> Noise {
        Noise::new(self.next_u64())
    }

    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// A number in [0, 1).
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// A normal number: mean 0, standard deviation 1.
    pub fn standard_normal(&mut self) -> f64 {
        if let Some(z) = self.spare.take() {
            return z;
        }
        // 1 - uniform is in (0, 1], so the log is finite.
        let r = (-2.0 * (1.0 - self.uniform()).ln()).sqrt();
        let (s, c) = (std::f64::consts::TAU * self.uniform()).sin_cos();
        self.spare = Some(r * s);
        r * c
    }

    /// Gaussian noise with this standard deviation (0: none, and no number drawn).
    pub fn gauss(&mut self, stddev: f64) -> f64 {
        if stddev > 0.0 {
            stddev * self.standard_normal()
        } else {
            0.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_same_seed_repeats() {
        let (mut a, mut b) = (Noise::new(7), Noise::new(7));
        assert!((0..100).all(|_| a.gauss(1.0) == b.gauss(1.0)));
    }

    #[test]
    fn gaussian_has_the_right_spread() {
        let mut n = Noise::new(1);
        let samples: Vec<f64> = (0..200_000).map(|_| n.gauss(0.5)).collect();
        let mean = samples.iter().sum::<f64>() / samples.len() as f64;
        let var = samples.iter().map(|s| (s - mean).powi(2)).sum::<f64>() / samples.len() as f64;
        assert!(mean.abs() < 0.01, "mean {mean}");
        assert!((var.sqrt() - 0.5).abs() < 0.01, "stddev {}", var.sqrt());
    }
}
