use crate::ode::{Integrator, Model, ModelState, NullSettings};

pub type RK4Settings = NullSettings;

pub struct RK4<M: Model>
where
  for<'a> &'a M::State: IntoIterator<Item = &'a M::S>,
  for<'a> &'a mut M::State: IntoIterator<Item = &'a mut M::S>,
{
  dxdt: M::State,
  intermediate_state: M::State,
  k: [M::State; 4],
}

impl<M: Model> Integrator<M> for RK4<M>
where
  for<'a> &'a M::State: IntoIterator<Item = &'a M::S>,
  for<'a> &'a mut M::State: IntoIterator<Item = &'a mut M::S>,
{
  type Settings = RK4Settings;

  fn new(_: Self::Settings) -> Self {
    Self {
      dxdt: M::State::new(),
      intermediate_state: M::State::new(),
      k: [
        M::State::new(),
        M::State::new(),
        M::State::new(),
        M::State::new(),
      ],
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

    let mut last_multiplier: M::S = 0.0.into();
    let mut is_first = true;
    self.intermediate_state.zeros_as(state);
    for (k_idx, multiplier) in
      [Some(0.5), Some(0.5), Some(1.0), None].iter().enumerate()
    {
      let multiplier = multiplier.map(|v| v.into());
      model.derivative(
        if is_first {
          &*state
        } else {
          &self.intermediate_state
        },
        &mut self.dxdt,
        &(*time + last_multiplier * *time_step),
      );

      self.k[k_idx].zeros_as(state);

      for (((k_val, next_state), state), dxdt) in (&mut self.k[k_idx])
        .into_iter()
        .zip(&mut self.intermediate_state)
        .zip(&mut *state)
        .zip(&self.dxdt)
      {
        *k_val = *time_step * *dxdt;
        if let Some(multiplier) = multiplier {
          *next_state = *state + multiplier * *k_val;
        }
      }

      is_first = false;
      if let Some(multiplier) = multiplier {
        last_multiplier = multiplier;
      }
    }

    let two: M::S = 2.0.into();
    let sixth = (1.0 / 6.0).into();

    for ((((k_0, k_1), k_2), k_3), state) in (&self.k[0])
      .into_iter()
      .zip(&self.k[1])
      .zip(&self.k[2])
      .zip(&self.k[3])
      .zip(state)
    {
      *state += (*k_0 + two * *k_1 + two * *k_2 + *k_3) * sixth;
    }
  }
}
