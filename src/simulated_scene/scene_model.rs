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
  floor_height: f32, // TODO: generalize
}

#[derive(Clone)]
pub struct SceneModelState {
  pub positions: Vec<Vector3<f32>>,
  pub velocities: Vec<Vector3<f32>>,
}

impl SceneModel {
  pub fn new(
    sim_meshs: Vec<SimMesh>,
    params: SceneModelParams,
    floor_height: f32,
  ) -> Self {
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
      floor_height,
    }
  }

  pub fn initial_state(&self) -> SceneModelState {
    // TODO: take and use initial transforms/scale
    SceneModelState {
      positions: self
        .sim_meshs
        .iter()
        .map(|m| m.vertices_obj_space().iter())
        .flatten()
        .cloned()
        .collect(),
      velocities: self
        .sim_meshs
        .iter()
        .map(|m| m.vertices_obj_space().iter().map(|_| Vector3::zeros()))
        .flatten()
        .collect(),
    }
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

  // TODO: fix box
  type IntoIter = Box<dyn Iterator<Item = Self::Item> + 'a>;

  #[inline]
  fn into_iter(self) -> Self::IntoIter {
    Box::new(
      self
        .positions
        .iter()
        .map(|v| v.iter())
        .flatten()
        .chain(self.velocities.iter().map(|v| v.iter()).flatten()),
    )
  }
}

impl<'a> IntoIterator for &'a mut SceneModelState {
  type Item = &'a mut f32;

  // TODO: fix box
  type IntoIter = Box<dyn Iterator<Item = Self::Item> + 'a>;

  #[inline]
  fn into_iter(self) -> Self::IntoIter {
    Box::new(
      self
        .positions
        .iter_mut()
        .map(|v| v.iter_mut())
        .flatten()
        .chain(self.velocities.iter_mut().map(|v| v.iter_mut()).flatten()),
    )
  }
}

impl ModelState<f32> for SceneModelState {
  fn new() -> Self {
    SceneModelState {
      positions: Vec::new(),
      velocities: Vec::new(),
    }
  }

  fn zeros_as(&mut self, other: &Self) {
    self
      .positions
      .resize(other.positions.len(), Vector3::zeros());
    self
      .velocities
      .resize(other.velocities.len(), Vector3::zeros());
  }
}

impl Model for SceneModel {
  type S = f32;
  type State = SceneModelState;

  fn derivative(&self, x: &Self::State, dxdt: &mut Self::State, _: &Self::S) {
    for ([start, end], mesh) in
      self.mesh_intervals.iter().zip(self.sim_meshs.iter())
    {
      let start = *start as usize;
      let end = *end as usize;
      let accels = mesh.vertex_accels(
        &x.positions[start..end],
        &x.velocities[start..end],
        &x.positions[start..end]
          .iter()
          .map(|pos| {
            let y_force = if pos[1] < self.floor_height {
              10000.0 * (self.floor_height - pos[1])
            } else {
              0.0
            };

            Vector3::new(0.0, y_force, 0.0)
          })
          .collect::<Vec<_>>(),
        self.params.g,
      );

      dxdt.velocities[start..end].copy_from_slice(&accels);
      dxdt.positions[start..end].copy_from_slice(&x.velocities[start..end]);
    }
  }
}
