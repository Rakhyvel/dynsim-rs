#![allow(dead_code)]
use nalgebra::Vector2;

use crate::state::State;

/// A particle's got state, and acceleration
#[derive(Default, Clone)]
pub struct Particle {
    pub state: State,
    pub accel: Vector2<f64>,
}
