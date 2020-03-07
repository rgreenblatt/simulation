use alga::general::RealField;
use ndarray::{Array, Ix1};

// TODO: generic State
pub type State<M> = Array<<M as Model>::S, Ix1>;

pub trait Model {
  type S: ndarray::ScalarOperand + RealField + From<f64>;

  fn derivative(&self, x: &State<Self>, dxdt: &mut State<Self>, t: &Self::S);
}
