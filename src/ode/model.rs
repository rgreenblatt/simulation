use alga::general::RealField;

pub trait ModelState<S>
where
  Self: Clone,
  for<'a> &'a Self: IntoIterator<Item = &'a S>,
  for<'a> &'a mut Self: IntoIterator<Item = &'a mut S>,
{
  fn new() -> Self;
  fn zeros_as(&mut self, other: &Self);
}

pub trait Model
where
  for<'a> &'a Self::State: IntoIterator<Item = &'a Self::S>,
  for<'a> &'a mut Self::State: IntoIterator<Item = &'a mut Self::S>,
{
  type S: RealField + From<f32>;

  type State: ModelState<Self::S>;

  fn derivative(&self, x: &Self::State, dxdt: &mut Self::State, t: &Self::S);
}
