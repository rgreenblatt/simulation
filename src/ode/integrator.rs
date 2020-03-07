use crate::ode::Model;

pub trait Integrator<M: Model>
where
  for<'a> &'a M::State: IntoIterator<Item = &'a M::S>,
  for<'a> &'a mut M::State: IntoIterator<Item = &'a mut M::S>,
{
  type Settings: Clone;

  fn new(settings: Self::Settings) -> Self;

  fn step_internal(
    &mut self,
    model: &M,
    state: &mut M::State,
    time: &M::S,
    time_step: &M::S,
  );

  fn step(
    &mut self,
    model: &M,
    state: &mut M::State,
    time: &mut M::S,
    time_step: &M::S,
  ) {
    self.step_internal(model, state, time, time_step);

    *time += *time_step;
  }

  fn n_steps(
    &mut self,
    model: &M,
    state: &mut M::State,
    time: &mut M::S,
    time_step: &M::S,
    steps: usize,
  ) {
    for _ in 0..steps {
      self.step(model, state, time, time_step);
    }
  }
}
