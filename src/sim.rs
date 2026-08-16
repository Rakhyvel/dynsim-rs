use std::f64::consts::PI;

use nalgebra::Vector2;
use rand::{RngExt, SeedableRng, rngs::StdRng};
use rayon::iter::{IntoParallelIterator, ParallelIterator};

type Vec2 = Vector2<f64>;

#[derive(Clone)]
pub struct Sim {
    pos: Vec<Vec2>,
    vel: Vec<Vec2>,
    acc: Vec<Vec2>,

    width: usize,
    height: usize,
    /// RGBA8 framebuffer, matching macroquad's `Image::bytes` layout.
    pixels: Vec<u8>,
}

impl Sim {
    pub fn new(num_particles: usize, seed: u64, width: usize, height: usize) -> Self {
        let mut rng = StdRng::seed_from_u64(seed);

        let mut pos = vec![];
        let mut vel = vec![];
        let mut acc = vec![];

        for _ in 0..num_particles {
            pos.push(Vec2::new(
                100.0 * (rng.random_range(-1.0..=1.0) - 3.4),
                100.0 * (rng.random_range(-1.0..=1.0) - 3.4),
            ));
            vel.push(Vec2::new(10.0, 0.0));
            acc.push(Vec2::new(0.0, 0.0));
        }

        for _ in 0..num_particles {
            pos.push(Vec2::new(
                100.0 * (rng.random_range(-1.0..=1.0)),
                100.0 * (rng.random_range(-1.0..=1.0)),
            ));
            vel.push(Vec2::new(0.0, 0.0));
            acc.push(Vec2::new(0.0, 0.0));
        }

        Self {
            pos,
            vel,
            acc,
            width,
            height,
            pixels: vec![0; width * height * 4],
        }
    }

    pub fn pixels(&self) -> &[u8] {
        &self.pixels
    }

    pub fn update(&mut self) {
        const MU: f64 = 200.0;
        const K: f64 = 5.0; // kinda chosen arbitarily, to counter gravitational forces at max contact
        const R: f64 = 5.0;

        // Derive restitution
        let e: f64 = 0.4; // elasticity, I think
        let l = e.ln();
        let zeta = -l / (PI * PI + l * l).sqrt();
        let c: f64 = zeta * (2.0 * K).sqrt();

        // Find each particles acceleration wrt every other particle
        let n = self.pos.len();
        self.acc = (0..n - 1)
            .into_par_iter()
            .fold(
                || vec![Vec2::zeros(); n],
                |mut local, i| {
                    let mut acc_i = Vec2::zeros();
                    for j in i + 1..n {
                        let p_i = &self.pos[i];
                        let v_i = &self.vel[i];
                        let p_j = &self.pos[j];
                        let v_j = &self.vel[j];

                        // Calculate the acceleration between these two particles with inverse square law
                        let to_other = p_j - p_i;
                        let d2 = to_other.magnitude_squared();
                        let d = d2.sqrt();
                        let n_hat: Vec2 = to_other / d;
                        let delta = 2.0 * R - d;

                        let v_rel: Vec2 = v_i - v_j;
                        let v_n = v_rel.dot(&n_hat); // >0 means approaching

                        let gravity: Vec2 = if delta > 0.0 {
                            // interior, use a softened gravity
                            MU * d / (2.0 * R).powi(3)
                        } else {
                            MU / d2
                        } * n_hat;

                        let dashpot = if delta > 0.0 {
                            -(K * delta + c * v_n)
                        } else {
                            0.0
                        } * n_hat;

                        acc_i += gravity + dashpot;
                        local[j] -= gravity + dashpot;
                    }

                    local[i] += acc_i;

                    local
                },
            )
            .reduce(
                || vec![Vec2::zeros(); n],
                |mut a, b| {
                    for (x, y) in a.iter_mut().zip(&b) {
                        *x += y;
                    }
                    a
                },
            );

        // Integrate, reset accel to zero
        const DT: f64 = 0.01;
        for i in 0..self.pos.len() {
            let accel = self.acc[i] * DT;
            self.vel[i] += accel;
            let vel = self.vel[i] * DT;
            self.pos[i] += vel;
        }
    }

    pub fn render(&mut self) {
        self.pixels.fill(0);
        // fill(0) leaves alpha at 0; set it back to opaque black.
        for px in self.pixels.chunks_exact_mut(4) {
            px[3] = 255;
        }

        let half_w = self.width as f64 / 2.0;
        let half_h = self.height as f64 / 2.0;

        for pos in &self.pos {
            let x = (pos.x + half_w) as isize;
            let y = (pos.y + half_h) as isize;

            if x < 0 || x >= self.width as isize || y < 0 || y >= self.height as isize {
                continue;
            }

            let idx = (y as usize * self.width + x as usize) * 4;

            self.pixels[idx..idx + 4].copy_from_slice(&[255, 255, 255, 255]);
        }
    }
}
