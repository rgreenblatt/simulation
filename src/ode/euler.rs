use crate::ode::{Integrator, Model, State};
use ndarray::azip;

pub struct Euler<M: Model> {
  dxdt: State<M>,
}

impl<M: Model> Integrator<M> for Euler<M> {
  fn step_internal(
    &mut self,
    model: &M,
    state: &mut State<M>,
    time: &M::S,
    time_step: &M::S,
  ) {
    model.derivative(state, &mut self.dxdt, time);
    azip!((state in state, dxdt in &self.dxdt) {*state += *dxdt * *time_step});
  }
}
