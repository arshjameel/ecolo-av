//! A small, fast, seedable random source: xoshiro256++ and Box-Muller.
//!
//! Copied from `aav_mppi_lag`'s `mppi-lag/src/rng.rs` (ours), plus [`Rng::below`] and
//! [`Rng::shuffle`]: no dependencies, so it builds offline, and a seed gives the same run.

/// xoshiro256++ (Blackman and Vigna, public domain), seeded through SplitMix64.
#[derive(Debug, Clone)]
pub struct Rng {
    s: [u64; 4],
    spare: Option<f64>,
}

fn splitmix(x: &mut u64) -> u64 {
    *x = x.wrapping_add(0x9e37_79b9_7f4a_7c15);
    let mut z = *x;
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

impl Rng {
    /// A generator from `seed`: the same seed, the same numbers.
    #[must_use]
    pub fn new(seed: u64) -> Self {
        let mut x = seed;
        Self {
            s: [
                splitmix(&mut x),
                splitmix(&mut x),
                splitmix(&mut x),
                splitmix(&mut x),
            ],
            spare: None,
        }
    }

    /// The next 64 random bits.
    pub fn next_u64(&mut self) -> u64 {
        let s = &mut self.s;
        let out = s[0].wrapping_add(s[3]).rotate_left(23).wrapping_add(s[0]);
        let t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = s[3].rotate_left(45);
        out
    }

    /// Uniform in (0, 1]: never 0, so its log is finite.
    pub fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) + 1) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// A standard normal (mean 0, sd 1), Box-Muller, two at a time.
    pub fn normal(&mut self) -> f64 {
        if let Some(z) = self.spare.take() {
            return z;
        }
        let r = (-2.0 * self.uniform().ln()).sqrt();
        let theta = std::f64::consts::TAU * self.uniform();
        self.spare = Some(r * theta.sin());
        r * theta.cos()
    }

    /// A whole number in `0..n` (`n` above 0), every one as likely (Lemire's method).
    pub fn below(&mut self, n: usize) -> usize {
        let n = n as u64;
        loop {
            let m = u128::from(self.next_u64()) * u128::from(n);
            let low = m as u64;
            if low >= n.wrapping_neg() % n {
                return (m >> 64) as usize;
            }
        }
    }

    /// Puts `items` in a random order (Fisher-Yates).
    pub fn shuffle<T>(&mut self, items: &mut [T]) {
        for i in (1..items.len()).rev() {
            let j = self.below(i + 1);
            items.swap(i, j);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normals_have_mean_0_and_sd_1() {
        let mut rng = Rng::new(7);
        let n = 200_000;
        let xs: Vec<f64> = (0..n).map(|_| rng.normal()).collect();
        let mean = xs.iter().sum::<f64>() / n as f64;
        let var = xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64;
        assert!(mean.abs() < 0.01, "{mean}");
        assert!((var - 1.0).abs() < 0.02, "{var}");
    }

    #[test]
    fn same_seed_same_numbers() {
        let (mut a, mut b) = (Rng::new(3), Rng::new(3));
        assert!((0..10).all(|_| a.next_u64() == b.next_u64()));
    }

    #[test]
    fn below_covers_its_range_evenly() {
        let mut rng = Rng::new(5);
        let mut counts = [0usize; 7];
        for _ in 0..70_000 {
            counts[rng.below(7)] += 1;
        }
        assert!(
            counts.iter().all(|&c| (9_000..11_000).contains(&c)),
            "{counts:?}"
        );
    }

    #[test]
    fn shuffle_keeps_every_item() {
        let mut rng = Rng::new(1);
        let mut items: Vec<usize> = (0..100).collect();
        rng.shuffle(&mut items);
        assert_ne!(items, (0..100).collect::<Vec<_>>());
        items.sort_unstable();
        assert_eq!(items, (0..100).collect::<Vec<_>>());
    }
}
