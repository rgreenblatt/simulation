use crate::ode::{Integrator, Model, ModelState, NullSettings};

pub type EulerSettings = NullSettings;

pub struct Euler<M: Model>
where
  for<'a> &'a M::State: IntoIterator<Item = &'a M::S>,
  for<'a> &'a mut M::State: IntoIterator<Item = &'a mut M::S>,
{
  dxdt: M::State,
}

impl<M: Model> Integrator<M> for Euler<M>
where
  for<'a> &'a M::State: IntoIterator<Item = &'a M::S>,
  for<'a> &'a mut M::State: IntoIterator<Item = &'a mut M::S>,
{
  type Settings = EulerSettings;

  fn new(_: Self::Settings) -> Self {
    Self {
      dxdt: M::State::new(),
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

    model.derivative(state, &mut self.dxdt, time);

    for (state, dxdt) in state.into_iter().zip(self.dxdt.into_iter()) {
      *state += *dxdt * *time_step
    }
  }
}
