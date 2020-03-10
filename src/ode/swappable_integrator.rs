use crate::ode::{
  Euler, EulerSettings, Integrator, Midpoint, MidpointSettings, Model,
  RK4Settings, RK4,
};
use clap::Clap;

#[derive(Clap, Clone, Debug)]
#[clap(about = "integrator type and args")]
pub enum IntegratorType {
  Euler(EulerSettings),
  Midpoint(MidpointSettings),
  RK4(RK4Settings),
}

pub enum SwappableIntegrator<M: Model>
where
  for<'a> &'a M::State: IntoIterator<Item = &'a M::S>,
  for<'a> &'a mut M::State: IntoIterator<Item = &'a mut M::S>,
{
  Euler(Euler<M>),
  Midpoint(Midpoint<M>),
  RK4(RK4<M>),
}

// impl<M: Model> SwappableIntegrator<M>
// where
//   for<'a> &'a M::State: IntoIterator<Item = &'a M::S>,
//   for<'a> &'a mut M::State: IntoIterator<Item = &'a mut M::S>,
// {
// }

impl<M: Model> Integrator<M> for SwappableIntegrator<M>
where
  for<'a> &'a M::State: IntoIterator<Item = &'a M::S>,
  for<'a> &'a mut M::State: IntoIterator<Item = &'a mut M::S>,
{
  type Settings = IntegratorType;

  fn new(t: IntegratorType) -> Self {
    match t {
      IntegratorType::Euler(settings) => Self::Euler(Euler::new(settings)),
      IntegratorType::Midpoint(settings) => {
        Self::Midpoint(Midpoint::new(settings))
      }
      IntegratorType::RK4(settings) => Self::RK4(RK4::new(settings)),
    }
  }

  fn step_internal(
    &mut self,
    model: &M,
    state: &mut M::State,
    time: &M::S,
    time_step: &M::S,
  ) {
    // TODO: use dispatch somehow (builder...)
    match self {
      Self::Euler(method) => {
        method.step_internal(model, state, time, time_step)
      }
      Self::Midpoint(method) => {
        method.step_internal(model, state, time, time_step)
      }
      Self::RK4(method) => method.step_internal(model, state, time, time_step),
    }
  }
}
