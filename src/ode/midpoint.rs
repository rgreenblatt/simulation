use crate::ode::{Integrator, Model, State};
use ndarray::azip;

pub struct Midpoint<M: Model> {
  dxdt: State<M>,
  midpoint_state: State<M>,
}

impl<M: Model> Integrator<M> for Midpoint<M> {
  fn step_internal(
    &mut self,
    model: &M,
    state: &mut State<M>,
    time: &M::S,
    time_step: &M::S,
  ) {
    model.derivative(state, &mut self.dxdt, time);

    let half_time_step = *time_step * 0.5.into();

    azip!(
      (state in &mut self.midpoint_state, dxdt in &self.dxdt)
      {*state += *dxdt * half_time_step}
    );

    let midpoint_time = *time + half_time_step;

    model.derivative(&self.midpoint_state, &mut self.dxdt, &midpoint_time);

    azip!(
      (state in state, dxdt in &self.dxdt)
      {*state += *dxdt * *time_step}
    );
  }
}
