use std::f64::consts::PI;

use nalgebra::Vector2;
use rand::{RngExt, SeedableRng, rngs::StdRng};

use crate::{particle::Particle, state::State};

type Vec2 = Vector2<f64>;

#[derive(Clone)]
pub struct Sim {
    particles: Vec<Particle>,

    width: usize,
    height: usize,
    /// RGBA8 framebuffer, matching macroquad's `Image::bytes` layout.
    pixels: Vec<u8>,
}

impl Sim {
    pub fn new(num_particles: usize, seed: u64, width: usize, height: usize) -> Self {
        let mut rng = StdRng::seed_from_u64(seed);

        let mut particles = vec![];

        for _ in 0..num_particles {
            let pos = Vector2::new(
                rng.random_range(-1.0..=1.0) - 1.4,
                rng.random_range(-1.0..=1.0) - 1.4,
            ) * 100.0;

            particles.push(Particle {
                state: State {
                    pos,
                    vel: Vector2::new(10.0, 0.0),
                },
                ..Default::default()
            });
        }

        for _ in 0..num_particles {
            let pos =
                Vector2::new(rng.random_range(-1.0..=1.0), rng.random_range(-1.0..=1.0)) * 100.0;

            particles.push(Particle {
                state: State {
                    pos,
                    vel: Vector2::new(0.0, 0.0),
                },
                ..Default::default()
            });
        }

        Self {
            particles,
            width,
            height,
            pixels: vec![0; width * height * 4],
        }
    }

    pub fn pixels(&self) -> &[u8] {
        &self.pixels
    }

    pub fn update(&mut self) {
        // Find each particles acceleration wrt every other particle
        for i in 0..(self.particles.len() - 1) {
            for j in (i + 1)..self.particles.len() {
                let this_particle = &self.particles[i];
                let other_particle = &self.particles[j];

                const MU: f64 = 200.0;
                const K: f64 = 3.0; // kinda chosen arbitarily, to counter gravitational forces at max contact
                const R: f64 = 5.0;

                // Derive restitution
                let e: f64 = 0.4; // elasticity, I think
                let l = e.ln();
                let zeta = -l / (PI * PI + l * l).sqrt();
                let c: f64 = zeta * (2.0 * K).sqrt();

                // Calculate the acceleration between these two particles with inverse square law
                let to_other = other_particle.state.pos - this_particle.state.pos;
                let n_hat: Vec2 = to_other.normalize();
                let d = to_other.magnitude();
                let d2 = d * d;
                let delta = 2.0 * R - d;

                let v_rel: Vec2 = this_particle.state.vel - other_particle.state.vel;
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

                self.particles[i].accel += gravity + dashpot;
                self.particles[j].accel -= gravity + dashpot;
            }
        }

        // Integrate, reset accel to zero
        const DT: f64 = 0.01;
        for i in 0..self.particles.len() {
            let this_particle = &mut self.particles[i];
            this_particle.state.vel += this_particle.accel * DT;
            this_particle.state.pos += this_particle.state.vel * DT;
            this_particle.accel = Vector2::new(0.0, 0.0);
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

        for p in &self.particles {
            let x = (p.state.pos.x + half_w) as isize;
            let y = (p.state.pos.y + half_h) as isize;

            if x < 0 || x >= self.width as isize || y < 0 || y >= self.height as isize {
                continue;
            }

            let idx = (y as usize * self.width + x as usize) * 4;

            self.pixels[idx..idx + 4].copy_from_slice(&[255, 255, 255, 255]);
        }
    }
}
