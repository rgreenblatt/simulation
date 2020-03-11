use crate::{
  ode::{Model, ModelState},
  simulated_scene::{SimMesh, S},
};
use nalgebra::base::iter::{MatrixIter, MatrixIterMut};
use nalgebra::dimension::*;
use nalgebra::storage::Owned;
use nalgebra::Vector3;
use std::iter::{Chain, Flatten, Map};
use std::slice::{Iter, IterMut};

#[derive(Debug, Clone)]
pub struct SceneModelParams {
  pub g: S,
}

#[derive(Clone)]
pub struct SceneModel {
  sim_meshs: Vec<SimMesh>,
  params: SceneModelParams,
  mesh_intervals: Vec<[u16; 2]>,
  floor_height: S, // TODO: generalize
}

#[derive(Clone)]
pub struct SceneModelState {
  pub positions: Vec<Vector3<S>>,
  pub velocities: Vec<Vector3<S>>,
}

impl SceneModel {
  pub fn new(
    sim_meshs: Vec<SimMesh>,
    params: SceneModelParams,
    floor_height: S,
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

  pub fn floor_height(&self) -> S {
    self.floor_height
  }
}

type BaseIntoIterGen<'a, I, M> = Flatten<Map<I, M>>;

fn float_iter_vector(
  vec: &Vector3<S>,
) -> MatrixIter<S, U3, U1, Owned<S, U3, U1>> {
  vec.iter()
}

type FloatIterVector =
  for<'a> fn(&'a Vector3<S>) -> MatrixIter<'a, S, U3, U1, Owned<S, U3, U1>>;

type BaseIntoIter<'a> =
  BaseIntoIterGen<'a, Iter<'a, Vector3<S>>, FloatIterVector>;

impl<'a> IntoIterator for &'a SceneModelState {
  type Item = &'a S;

  type IntoIter = Chain<BaseIntoIter<'a>, BaseIntoIter<'a>>;

  #[inline]
  fn into_iter(self) -> Self::IntoIter {
    self
      .positions
      .iter()
      .map(float_iter_vector as FloatIterVector)
      .flatten()
      .chain(
        self
          .velocities
          .iter()
          .map(float_iter_vector as FloatIterVector)
          .flatten(),
      )
  }
}

fn float_iter_vector_mut(
  vec: &mut Vector3<S>,
) -> MatrixIterMut<S, U3, U1, Owned<S, U3, U1>> {
  vec.iter_mut()
}

type FloatIterVectorMut =
  for<'a> fn(
    &'a mut Vector3<S>,
  ) -> MatrixIterMut<'a, S, U3, U1, Owned<S, U3, U1>>;

type BaseIntoIterMut<'a> =
  BaseIntoIterGen<'a, IterMut<'a, Vector3<S>>, FloatIterVectorMut>;

impl<'a> IntoIterator for &'a mut SceneModelState {
  type Item = &'a mut S;

  type IntoIter = Chain<BaseIntoIterMut<'a>, BaseIntoIterMut<'a>>;

  #[inline]
  fn into_iter(self) -> Self::IntoIter {
    self
      .positions
      .iter_mut()
      .map(float_iter_vector_mut as FloatIterVectorMut)
      .flatten()
      .chain(
        self
          .velocities
          .iter_mut()
          .map(float_iter_vector_mut as FloatIterVectorMut)
          .flatten(),
      )
  }
}

impl ModelState<S> for SceneModelState {
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
  type S = S;
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
