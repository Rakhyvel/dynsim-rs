use std::f64::consts::{PI, TAU};

use nalgebra::Vector2;
use rand::{Rng, RngExt, SeedableRng, rngs::StdRng};
use rayon::iter::{IntoParallelIterator, ParallelIterator};

type Vec2 = Vector2<f64>;

#[derive(Clone)]
pub struct Sim {
    pos: Vec<Vec2>,
    vel: Vec<Vec2>,
    acc: Vec<Vec2>,

    color: Vec<f64>,
    color_vel: Vec<f64>,

    width: usize,
    height: usize,
    /// RGBA8 framebuffer, matching macroquad's `Image::bytes` layout.
    pixels: Vec<u8>,

    focus: Vec2,
}

impl Sim {
    const MU: f64 = 100.0;

    pub fn new(num_particles: usize, seed: u64, width: usize, height: usize) -> Self {
        let mut rng = StdRng::seed_from_u64(seed);

        let (pos, vel, color) = Self::setup_smash(num_particles, &mut rng);

        Self {
            pos,
            vel,
            acc: (0..num_particles).map(|_| Vec2::zeros()).collect(),
            color,
            color_vel: (0..num_particles).map(|_| 0.0f64).collect(),
            width,
            height,
            pixels: vec![0; width * height * 4],
            focus: Vec2::zeros(),
        }
    }

    fn setup_smash(num_particles: usize, rng: &mut impl Rng) -> (Vec<Vec2>, Vec<Vec2>, Vec<f64>) {
        let mut pos = vec![];
        let mut vel = vec![];
        let mut color = vec![];

        const ENTER: f64 = 100.0;
        const SPEED: f64 = 200.0;
        const MISS: f64 = 0.2;
        const COMP: f64 = 0.5;

        for _ in 0..(COMP * num_particles as f64) as usize {
            pos.push(Vec2::new(
                20.0 * (rng.random_range(-1.0..=1.0) + ENTER),
                20.0 * (rng.random_range(-1.0..=1.0) + MISS),
            ));
            vel.push(Vec2::new(-SPEED, 0.0));
            color.push(-0.0);
        }
        for _ in 0..((1.0 - COMP) * num_particles as f64) as usize {
            pos.push(Vec2::new(
                20.0 * (rng.random_range(-1.0..=1.0) - ENTER),
                20.0 * (rng.random_range(-1.0..=1.0) - MISS),
            ));
            vel.push(Vec2::new(SPEED, 0.0));
            color.push(0.0);
        }

        (pos, vel, color)
    }

    fn setup_protodisk(
        num_particles: usize,
        rng: &mut impl Rng,
    ) -> (Vec<Vec2>, Vec<Vec2>, Vec<f64>) {
        const RADII: f64 = 100.0;

        // create a vec of random (r, theta) points, sorted by r
        let mut pos_r: Vec<(f64, f64)> = (0..num_particles)
            .map(|_| {
                let r = rng.random_range(0.0..1.0) * RADII;
                let theta = rng.random_range(0.0..TAU);
                (r, theta)
            })
            .collect();
        pos_r.sort_by(|(r1, _), (r2, _)| r1.total_cmp(r2));

        (
            // pos: convert radial to cartesian
            pos_r
                .iter()
                .map(|(r, theta)| Vec2::new(theta.cos() * r, theta.sin() * r))
                .collect(),
            // vel: give particles more circular orbits as they get further out
            pos_r
                .iter()
                .enumerate()
                .map(|(k, (r, theta))| {
                    let v_circ = (Self::MU * k as f64 / r).sqrt();
                    Vec2::new(v_circ * -theta.sin(), v_circ * theta.cos())
                })
                .collect(),
            // blank color
            (0..num_particles).map(|_| 0.0).collect(),
        )
    }

    pub fn pixels(&self) -> &[u8] {
        &self.pixels
    }

    pub fn update(&mut self) {
        const K: f64 = 2.0; // kinda chosen arbitarily, to counter gravitational forces at max contact
        const R: f64 = 6.5;

        // Derive restitution
        let e: f64 = 0.4; // elasticity, I think
        let l = e.ln();
        let zeta = -l / (PI * PI + l * l).sqrt();
        let c: f64 = zeta * (2.0 * K).sqrt();

        // Find each particles acceleration wrt every other particle
        let n = self.pos.len();
        let (acc, color_acc) = (0..n - 1)
            .into_par_iter()
            .fold(
                || (vec![Vec2::zeros(); n], vec![0.0f64; n]),
                |mut local, i| {
                    let mut acc_i = Vec2::zeros();
                    let mut color_acc_i: f64 = 0.0;
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
                            Self::MU * d / (2.0 * R).powi(3)
                        } else {
                            Self::MU / d2 // cant be div by zero, otherwise delta would be positive
                        } * n_hat;

                        let dashpot = if delta > 0.0 {
                            -(K * delta + c * v_n)
                        } else {
                            0.0
                        } * n_hat;

                        let color_f = if delta > 0.0 {
                            const K_C: f64 = 4.0;
                            const C_C: f64 = 0.05;
                            const GAIN: f64 = 0.5;
                            K_C * (self.color[j] - self.color[i])
                                + C_C * (self.color_vel[j] - self.color_vel[i])
                                + GAIN * v_n.max(0.0).powi(2)
                        } else {
                            0.0
                        };

                        acc_i += gravity + dashpot;
                        local.0[j] -= gravity + dashpot;

                        color_acc_i += color_f;
                        local.1[j] -= color_f;
                    }

                    local.0[i] += acc_i;
                    local.1[i] += color_acc_i;

                    local
                },
            )
            .reduce(
                || (vec![Vec2::zeros(); n], vec![0.0f64; n]),
                |(mut a, mut color_a), (b, color_b)| {
                    for (x, y) in a.iter_mut().zip(&b) {
                        *x += y;
                    }
                    for (x, y) in color_a.iter_mut().zip(&color_b) {
                        *x += y;
                    }
                    (a, color_a)
                },
            );
        self.acc = acc;

        // Integrate, reset accel to zero
        const DT: f64 = 0.01;
        for i in 0..self.pos.len() {
            let accel = self.acc[i] * DT;
            self.vel[i] += accel;
            let vel = self.vel[i] * DT;
            self.pos[i] += vel;

            const K0: f64 = 3.0;
            const C0: f64 = 0.2;
            let color_accel = color_acc[i] - K0 * self.color[i] - C0 * self.color_vel[i];
            self.color_vel[i] += color_accel * DT;
            let color_vel = self.color_vel[i];
            self.color[i] += color_vel * DT;
        }
    }

    pub fn render(&mut self) {
        const WINDOW: f64 = 100.0;
        for _ in 0..3 {
            let mut sum = Vec2::zeros();
            let mut count: usize = 0;
            for p in &self.pos {
                if p.metric_distance(&self.focus) < WINDOW {
                    sum += p;
                    count += 1;
                }
            }
            if count > 1 {
                self.focus = sum / count as f64;
            }
        }

        self.pixels.fill(0);
        // fill(0) leaves alpha at 0; set it back to opaque black.
        for px in self.pixels.chunks_exact_mut(4) {
            px[3] = 255;
        }

        let half_w = self.width as f64 / 2.0;
        let half_h = self.height as f64 / 2.0;

        const ZOOM: f64 = 1.0;
        const PIXEL: usize = 4;

        for (pos, color) in self.pos.iter().zip(&self.color) {
            let x = ((pos.x - self.focus.x) * ZOOM + half_w) as isize;
            let y = ((pos.y - self.focus.y) * ZOOM + half_h) as isize;

            if x < 0 || x >= self.width as isize || y < 0 || y >= self.height as isize {
                continue;
            }

            let color = color.clamp(-1.0, 1.0);

            let r: u8 = (255.0 * (1.0 + color).min(1.0)) as u8;
            let g: u8 = (255.0 * (1.0 - color.abs())) as u8;
            let b: u8 = (255.0 * (1.0 - color).min(1.0)) as u8;

            for i in 0..PIXEL {
                for j in 0..PIXEL {
                    let px = x as usize + i;
                    let py = y as usize + j;
                    if px >= self.width || py >= self.height {
                        continue;
                    }
                    let idx = (py * self.width + px) * 4;
                    self.pixels[idx..idx + 4].copy_from_slice(&[r, g, b, 255]);
                }
            }
        }
    }
}
