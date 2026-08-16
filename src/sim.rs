use nalgebra::Vector2;
use rand::RngExt;

use crate::{particle::Particle, state::State};

pub struct Sim {
    particles: Vec<Particle>,

    width: usize,
    height: usize,
    /// RGBA8 framebuffer, matching macroquad's `Image::bytes` layout.
    pixels: Vec<u8>,
}

impl Sim {
    pub fn new(num_particles: usize, width: usize, height: usize) -> Self {
        let mut rng = rand::rng();

        let mut particles = vec![];

        for _ in 0..num_particles {
            let pos =
                Vector2::new(rng.random_range(-1.0..=1.0), rng.random_range(-1.0..=1.0)) * 500.0;

            particles.push(Particle {
                state: State {
                    pos,
                    ..Default::default()
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
        for i in 0..self.particles.len() {
            for j in 0..self.particles.len() {
                if i == j {
                    // No self-attraction
                    continue;
                }

                let this_particle = &self.particles[i];
                let other_particle = &self.particles[j];

                // Calculate the acceleration between these two particles with inverse square law
                const MU: f64 = 200.0;
                let to_other = other_particle.state.pos - this_particle.state.pos;
                let to_other_norm = to_other.normalize();
                let vel_diff = other_particle.state.vel - this_particle.state.vel;
                let dist_squared = to_other.magnitude_squared();
                let speed_diff = to_other_norm.dot(&vel_diff);

                let factor: f64 = if dist_squared < 100.0 {
                    // Add repulsive force
                    speed_diff - 0.2 * (100.0 - dist_squared)
                } else {
                    MU / dist_squared
                };
                self.particles[i].accel += factor * to_other_norm;
            }
        }

        // Integrate, reset accel to zero
        for i in 0..self.particles.len() {
            let this_particle = &mut self.particles[i];
            this_particle.state.vel += this_particle.accel * 0.1;
            this_particle.state.pos += this_particle.state.vel * 0.1;
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
