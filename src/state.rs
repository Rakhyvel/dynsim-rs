use nalgebra::Vector2;
use std::ops::{Add, AddAssign, Mul};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct State {
    pub pos: Vector2<f64>,
    pub vel: Vector2<f64>,
}

impl State {
    pub fn new(pos: Vector2<f64>, vel: Vector2<f64>) -> Self {
        Self { pos, vel }
    }

    pub fn zero() -> Self {
        Self::default()
    }
}

impl Add for State {
    type Output = State;
    fn add(self, rhs: State) -> State {
        State::new(self.pos + rhs.pos, self.vel + rhs.vel)
    }
}

impl AddAssign for State {
    fn add_assign(&mut self, rhs: State) {
        self.pos += rhs.pos;
        self.vel += rhs.vel;
    }
}

impl Mul<State> for f64 {
    type Output = State;
    fn mul(self, rhs: State) -> State {
        State::new(rhs.pos * self, rhs.vel * self)
    }
}
