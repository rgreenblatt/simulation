use crate::{
  ode::{Model, ModelState},
  simulated_scene::SimMesh,
};
use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub struct SceneModelParams {
  pub g: f32,
}

#[derive(Clone)]
pub struct SceneModel {
  sim_meshs: Vec<SimMesh>,
  params: SceneModelParams,
  mesh_intervals: Vec<[u16; 2]>,
}

#[derive(Clone)]
pub struct SceneModelState {
  pub positions: Vec<Vector3<f32>>,
  pub velocities: Vec<Vector3<f32>>,
  check: Vec<f32>,
}

impl SceneModel {
  pub fn new(sim_meshs: Vec<SimMesh>, params: SceneModelParams) -> Self {
    let mut mesh_intervals = Vec::new();

    let mut total_size = 0;

    for mesh in &sim_meshs {
      let new_total_size = total_size + mesh.num_vertices();
      mesh_intervals.push([total_size, new_total_size]);
      total_size = new_total_size;
    }

    Self {
      sim_meshs,
      params,
      mesh_intervals,
    }
  }

  pub fn initial_state(&self) -> SceneModelState {
    unimplemented!()
  }

  pub fn meshs(&self) -> &[SimMesh] {
    &self.sim_meshs
  }

  pub fn mesh_intervals(&self) -> &[[u16; 2]] {
    &self.mesh_intervals
  }
}

impl<'a> IntoIterator for &'a SceneModelState {
  type Item = &'a f32;
  type IntoIter = std::slice::Iter<'a, f32>;

  fn into_iter(self) -> Self::IntoIter {
    self.check.iter()
  }
}

impl<'a> IntoIterator for &'a mut SceneModelState {
  type Item = &'a mut f32;
  type IntoIter = std::slice::IterMut<'a, f32>;

  fn into_iter(self) -> Self::IntoIter {
    self.check.iter_mut()
  }
}

impl ModelState<f32> for SceneModelState {
  fn new() -> Self {
    SceneModelState {
      positions: Vec::new(),
      velocities: Vec::new(),
      check: Vec::new(),
    }
  }
}

impl Model for SceneModel {
  type S = f32;
  type State = SceneModelState;

  fn derivative(&self, x: &Self::State, dxdt: &mut Self::State, _: &Self::S) {
    unimplemented!()
  }
}
