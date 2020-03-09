use crate::ode::{Integrator, Model, ModelState, NullSettings};

pub type MidpointSettings = NullSettings;

pub struct Midpoint<M: Model>
where
  for<'a> &'a M::State: IntoIterator<Item = &'a M::S>,
  for<'a> &'a mut M::State: IntoIterator<Item = &'a mut M::S>,
{
  dxdt: M::State,
  midpoint_state: M::State,
}

impl<M: Model> Integrator<M> for Midpoint<M>
where
  for<'a> &'a M::State: IntoIterator<Item = &'a M::S>,
  for<'a> &'a mut M::State: IntoIterator<Item = &'a mut M::S>,
{
  type Settings = MidpointSettings;

  fn new(_: Self::Settings) -> Self {
    Self {
      dxdt: M::State::new(),
      midpoint_state: M::State::new(),
    }
  }

  fn step_internal(
    &mut self,
    model: &M,
    state: &mut M::State,
    time: &M::S,
    time_step: &M::S,
  ) {
    self.dxdt.zeros_as(state);
    self.midpoint_state.zeros_as(state);

    model.derivative(state, &mut self.dxdt, time);

    let half_time_step = *time_step * 0.5.into();

    for ((midpoint_state, state), dxdt) in (&mut self.midpoint_state)
      .into_iter()
      .zip(state.into_iter())
      .zip(self.dxdt.into_iter())
    {
      *midpoint_state = *state + *dxdt * half_time_step
    }

    let midpoint_time = *time + half_time_step;

    model.derivative(&self.midpoint_state, &mut self.dxdt, &midpoint_time);

    for (state, dxdt) in state.into_iter().zip(self.dxdt.into_iter()) {
      *state += *dxdt * *time_step
    }
  }
}
