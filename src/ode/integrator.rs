use crate::ode::{Model, State};

pub trait Integrator<M: Model> {
  fn step_internal(
    &mut self,
    model: &M,
    state: &mut State<M>,
    time: &M::S,
    time_step: &M::S,
  );

  fn step(
    &mut self,
    model: &M,
    state: &mut State<M>,
    time: &mut M::S,
    time_step: &M::S,
  ) {
    self.step_internal(model, state, time, time_step);

    *time += *time_step;
  }

  // fn integrate<M: Model, F: FnMut(&State<M::S>)>(
  //   model: &M,
  //   state: &mut State<M::S>,
  //   start_time: f32,
  //   end_time: f32,
  //   time_step: f32,
  //   callback: &F,
  // ) {
  // }
}
