#[cfg(test)]
use crate::assert_float_eq;
use crate::LoadedMesh;
use nalgebra::{Matrix3, Point3, Vector3};
#[cfg(test)]
use nalgebra::{Rotation3, Translation3};
use std::collections::HashSet;
use std::iter::FromIterator;

#[cfg(test)]
use proptest::prelude::*;
#[cfg(test)]
use proptest_derive::Arbitrary;

#[derive(Debug, Clone, PartialEq)]
pub struct MeshParams {
  pub incompressibility: f32,
  pub rigidity: f32,

  pub viscous_incompressibility: f32,
  pub viscous_rigidity: f32,

  pub density: f32,
}

// TODO: collisions
#[derive(Clone)]
pub struct SimMesh {
  vertex_positions_obj_space: Vec<Vector3<f32>>, // per vertex
  vertex_mass: Vec<f32>,                         // per vertex

  tetras: Vec<[u16; 4]>, // per tet
  // scaled by face area
  opposite_normals: Vec<[Vector3<f32>; 4]>, // per tet
  inv_barycentric_mat: Vec<Matrix3<f32>>,   // per tet

  boundary_vertices: Vec<u16>,
  // indexing scheme must be the same as boundary_vertices
  boundary_faces: Vec<[u16; 3]>,

  params: MeshParams,
}

impl SimMesh {
  fn get_vertex(
    tetra: [u16; 4],
    vals: &[Vector3<f32>],
    idx: usize,
  ) -> Vector3<f32> {
    vals[tetra[idx] as usize]
  }

  fn tetra_val_edges(tetra: [u16; 4], vals: &[Vector3<f32>]) -> Matrix3<f32> {
    let get_vertex = |idx| Self::get_vertex(tetra, vals, idx);
    Matrix3::from_columns(&[
      get_vertex(0) - get_vertex(3),
      get_vertex(1) - get_vertex(3),
      get_vertex(2) - get_vertex(3),
    ])
  }

  pub fn new(
    (vertex_positions_obj_space, tetras): LoadedMesh,
    params: MeshParams,
  ) -> Self {
    // SPEED: reserve space
    let mut inv_barycentric_mat = Vec::new();
    let mut opposite_normals = Vec::new();
    let mut vertex_mass = vec![0.0; vertex_positions_obj_space.len()];

    let mut boundary_faces_set = HashSet::new();

    for tetra in &tetras {
      let edges = Self::tetra_val_edges(*tetra, &vertex_positions_obj_space);
      inv_barycentric_mat.push(
        // TODO: fix expect
        edges.try_inverse().expect(
          "all tetrahedrons should have inverses for barycentric coords",
        ),
      );

      // TODO: check use of columns is as expected
      let volume = (edges.column(0).cross(&edges.column(1)))
        .dot(&edges.column(2))
        .abs()
        / 6.0;

      for vertex_idx in tetra {
        vertex_mass[*vertex_idx as usize] += params.density * volume / 4.0;
      }

      let mut tet_opposite_normals = [Vector3::zeros(); 4];

      for ((face, other), opposite_normal) in [
        [tetra[1], tetra[2], tetra[3]],
        [tetra[0], tetra[2], tetra[3]],
        [tetra[0], tetra[1], tetra[3]],
        [tetra[0], tetra[1], tetra[2]],
      ]
      .iter_mut()
      .zip([tetra[0], tetra[1], tetra[2], tetra[3]].iter())
      .zip(tet_opposite_normals.iter_mut())
      {
        let get_vertex = |idx| vertex_positions_obj_space[idx as usize];

        face.sort();

        let vertices = [
          get_vertex(face[0]),
          get_vertex(face[1]),
          get_vertex(face[2]),
        ];

        let other_vertex = get_vertex(*other);

        // same magnitude as face area
        *opposite_normal =
          0.5 * (vertices[1] - vertices[0]).cross(&(vertices[2] - vertices[0]));

        let above_plane =
          (other_vertex - vertices[0]).dot(opposite_normal) > 0.0;

        if above_plane {
          *opposite_normal *= -1.0;

          face.reverse();
        }

        debug_assert_eq!(
          face.len(),
          HashSet::<u16>::from_iter(face.iter().cloned()).len()
        );

        if boundary_faces_set.contains(face) {
          boundary_faces_set.remove(face);
        } else {
          boundary_faces_set.insert(face.clone());
        }
      }

      opposite_normals.push(tet_opposite_normals);
    }

    let mut boundary_vertices = Vec::new();
    let mut vertex_idx_to_boundary_vertex_idx = Vec::new();
    vertex_idx_to_boundary_vertex_idx
      .resize(vertex_positions_obj_space.len(), None);

    let boundary_faces = boundary_faces_set
      .into_iter()
      .map(|boundary_face| {
        let mut new_boundary_face = [0; 3];
        for (i, vertex_idx) in boundary_face.iter().enumerate() {
          let vertex_idx = *vertex_idx as usize;
          new_boundary_face[i] = if let Some(boundary_vertex_idx) =
            vertex_idx_to_boundary_vertex_idx[vertex_idx]
          {
            boundary_vertex_idx
          } else {
            let boundary_vertex_idx = boundary_vertices.len() as u16;
            boundary_vertices.push(vertex_idx as u16);
            vertex_idx_to_boundary_vertex_idx[vertex_idx] =
              Some(boundary_vertex_idx);

            boundary_vertex_idx
          };
        }

        new_boundary_face
      })
      .collect();

    Self {
      vertex_positions_obj_space,
      vertex_mass,
      tetras,
      opposite_normals,
      inv_barycentric_mat,
      boundary_vertices,
      boundary_faces,
      params,
    }
  }

  pub fn num_vertices(&self) -> u16 {
    self.vertex_mass.len() as u16
  }

  pub fn vertices_obj_space(&self) -> &[Vector3<f32>] {
    &self.vertex_positions_obj_space
  }

  pub fn vertex_accels(
    &self,
    positions: &[Vector3<f32>],
    velocities: &[Vector3<f32>],
    forces: &[Vector3<f32>], // external forces other than g should be input
    g: f32,
  ) -> Vec<Vector3<f32>> {
    let mut forces = forces.to_vec();

    for ((tetra, opposite_normals), inv_barycentric_mat) in self
      .tetras
      .iter()
      .zip(self.opposite_normals.iter())
      .zip(self.inv_barycentric_mat.iter())
    {
      let compute_deformation_grad = |vals: &[Vector3<f32>]| {
        let val_edges = Self::tetra_val_edges(*tetra, vals);

        val_edges * inv_barycentric_mat
      };

      let deformation_grad = compute_deformation_grad(positions);
      let velocity_deformation_grad = compute_deformation_grad(velocities);

      let elastic_strain =
        deformation_grad.transpose() * deformation_grad - Matrix3::identity();

      let viscous_strain = deformation_grad.transpose()
        * velocity_deformation_grad
        + velocity_deformation_grad.transpose() * deformation_grad;

      let strain_to_stress =
        |strain: Matrix3<_>, incompressibility, rigidity| {
          incompressibility * Matrix3::identity() * strain.trace()
            + 2.0 * rigidity * strain
        };

      let elastic_stress = strain_to_stress(
        elastic_strain,
        self.params.incompressibility,
        self.params.rigidity,
      );

      let viscous_stress = strain_to_stress(
        viscous_strain,
        self.params.viscous_incompressibility,
        self.params.viscous_rigidity,
      );

      let stress = elastic_stress + viscous_stress;

      let mat = deformation_grad * stress;

      for (vertex_idx, opposite_normal) in
        tetra.iter().zip(opposite_normals.iter())
      {
        let force = mat * opposite_normal;

        forces[*vertex_idx as usize] += force;
      }
    }

    // gravity
    for (force, mass) in forces.iter_mut().zip(self.vertex_mass.iter()) {
      *force += -mass * g * Vector3::new(0.0, 1.0, 0.0);
    }

    forces
      .into_iter()
      .enumerate()
      .map(|(vertex_idx, force)| force / self.vertex_mass[vertex_idx])
      .collect()
  }

  // SPEED: consider changing to avoid copies
  pub fn boundary_vertices_faces(
    &self,
    positions: &[Vector3<f32>],
  ) -> (Vec<Point3<f32>>, Vec<Point3<u16>>) {
    (
      self
        .boundary_vertices
        .iter()
        .map(|vertex_idx| Point3::from(positions[*vertex_idx as usize]))
        .collect(),
      self
        .boundary_faces
        .iter()
        .map(|v| Point3::new(v[0], v[1], v[2]))
        .collect(),
    )
  }
}

#[cfg(test)]
fn basic_params() -> MeshParams {
  MeshParams {
    incompressibility: 1.0,
    rigidity: 1.0,
    viscous_incompressibility: 1.0,
    viscous_rigidity: 1.0,
    density: 1.0,
  }
}

#[cfg(test)]
type MeshInfo = (SimMesh, Vec<Vector3<f32>>, Vec<[u16; 4]>);

#[cfg(test)]
#[derive(Debug, Arbitrary)]
enum MeshOptions {
  SingleTet,
  DoubleTet,
}

#[cfg(test)]
impl MeshOptions {
  fn get_mesh(&self, params: &MeshParams) -> MeshInfo {
    match self {
      MeshOptions::SingleTet => {
        let positions = vec![
          Vector3::new(0.0, 0.0, 0.0),
          Vector3::new(1.0, 0.0, 0.0),
          Vector3::new(0.0, 1.0, 0.0),
          Vector3::new(0.0, 0.0, 1.0),
        ];
        let tetras = vec![[0, 1, 2, 3]];
        let mesh =
          SimMesh::new((positions.clone(), tetras.clone()), params.clone());

        (mesh, positions, tetras)
      }
      MeshOptions::DoubleTet => {
        let positions = vec![
          Vector3::new(0.0, 0.0, 0.0),
          Vector3::new(1.0, 0.0, 0.0),
          Vector3::new(0.0, 1.0, 0.0),
          Vector3::new(0.0, 0.0, 1.0),
          Vector3::new(1.0, 1.0, 0.0),
        ];
        let tetras = vec![[0, 1, 2, 3], [4, 1, 2, 3]];
        let mesh =
          SimMesh::new((positions.clone(), tetras.clone()), params.clone());

        (mesh, positions, tetras)
      }
    }
  }
}

#[test]
fn single_tet_basic() {
  let params = basic_params();
  let (mesh, positions, tetras) = MeshOptions::SingleTet.get_mesh(&params);

  assert_eq!(mesh.vertex_positions_obj_space, positions);
  assert_eq!(mesh.params, params);
  assert_eq!(mesh.tetras, tetras);

  // assert_eq!(
  //   HashSet::<[u16; 3]>::from_iter(mesh.boundary_faces.iter().cloned().map(
  //     // Sort to avoid ordering issues
  //     |mut v| {
  //       v.sort();
  //       v
  //     }
  //   )),
  //   HashSet::from_iter(
  //     [[1, 2, 3], [0, 2, 3], [0, 1, 3], [0, 1, 2],]
  //       .iter()
  //       .cloned()
  //   )
  // );
  // assert_eq!(mesh.boundary_vertices, vec![0, 1, 2, 3]);

  let vol = 1.0 / 6.0;
  let total_mass: f32 = vol * params.density;
  let actual_total_mass: f32 = mesh.vertex_mass.iter().sum();
  assert_float_eq!(total_mass, actual_total_mass);
  let each_mass = actual_total_mass / 4.0;
  for mass in &mesh.vertex_mass {
    assert_float_eq!(mass, each_mass);
  }
}

#[test]
fn double_tet_basic() {
  let params = basic_params();
  let (mesh, positions, tetras) = MeshOptions::DoubleTet.get_mesh(&params);

  assert_eq!(mesh.vertex_positions_obj_space, positions);
  assert_eq!(mesh.params, params);
  assert_eq!(mesh.tetras, tetras);

  // assert_eq!(
  //   HashSet::<[u16; 3]>::from_iter(mesh.boundary_faces.iter().cloned().map(
  //     // Sort to avoid ordering issues
  //     |mut v| {
  //       v.sort();
  //       v
  //     }
  //   )),
  //   HashSet::from_iter(
  //     [
  //       [0, 2, 3],
  //       [0, 1, 3],
  //       [0, 1, 2],
  //       [2, 3, 4],
  //       [1, 3, 4],
  //       [1, 2, 4],
  //     ]
  //     .iter()
  //     .cloned()
  //   )
  // );
  // assert_eq!(mesh.boundary_vertices, vec![0, 1, 2, 3, 4]);

  let vol_each_tet = 1.0 / 6.0;
  let vol = vol_each_tet * 2.0;
  let total_mass: f32 = vol * params.density;
  let actual_total_mass: f32 = mesh.vertex_mass.iter().sum();
  assert_float_eq!(total_mass, actual_total_mass);
  assert_float_eq!(mesh.vertex_mass[0], total_mass / 2.0 / 4.0);
  assert_float_eq!(mesh.vertex_mass[4], total_mass / 2.0 / 4.0);
  let each_mass = total_mass / 4.0;
  for mass in &mesh.vertex_mass[1..4] {
    assert_float_eq!(mass, each_mass);
  }
}

#[cfg(test)]
proptest! {
#[test]
fn rigid_transform_gravity(
  g in 0.0f32..100.0,
  incompressibility in 0.001f32..100.0,
  rigidity in 0.001f32..100.0,
  viscous_incompressibility in 0.001f32..100.0,
  viscous_rigidity in 0.001f32..100.0,
  density in 100.0f32..10000.0,
  mesh_option : MeshOptions,
  translation in prop::array::uniform3(-2.0f32..2.0),
  rotation in prop::array::uniform3(-2.0f32..2.0),
) {
  let params = MeshParams {
    incompressibility,
    rigidity,
    viscous_incompressibility,
    viscous_rigidity,
    density,
  };
  let (mesh, positions, _) = mesh_option.get_mesh(&params);

  let translation = Translation3::from(Vector3::from(translation));
  let rotation = Rotation3::new(Vector3::from(rotation));
  let transform = translation * rotation;

  let accels = mesh.vertex_accels(
    &positions.iter().map(|vertex| transform * vertex).collect::<Vec<_>>(),
    &vec![Vector3::new(0.0, 0.0, 0.0); positions.len()],
    &vec![Vector3::new(0.0, 0.0, 0.0); positions.len()],
    g,
  );

  for accel in accels {
    assert_float_eq!(accel[0], 0.0);
    assert_float_eq!(accel[1], -g);
    assert_float_eq!(accel[2], 0.0);
  }
}
}
