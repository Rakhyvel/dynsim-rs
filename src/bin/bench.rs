use std::time::Instant;

use dynsim::{HEIGHT, WIDTH, sim::Sim};

pub fn main() {
    for n in [500, 1000, 2000, 4000] {
        let mut sim = Sim::new(n, 42, WIDTH, HEIGHT);
        // warmup
        let t0 = Instant::now();
        for _ in 0..2000 {
            sim.update();
        }
        let elapsed = t0.elapsed();

        println!("{n}: {elapsed:?}",);
    }
}
