use crate::LoadedMesh;
use nalgebra::{Matrix3, Matrix4, Point3, Vector3};
use std::collections::HashSet;
use std::iter::FromIterator;

#[derive(Debug, Clone)]
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
  // not needed?
  vertex_positions_obj_space: Vec<Vector3<f32>>, // per vertex
  vertex_mass: Vec<f32>,                         // per vertex

  // TODO: WTF?
  inv_barycentric_4_mat: Vec<Matrix4<f32>>, // per tet
  inv_barycentric_3_mat: Vec<Matrix3<f32>>, // per tet
  tetra_volume: Vec<f32>,                   // per tet
  tetras: Vec<[u16; 4]>,                    // per tet

  boundary_vertices: Vec<u16>,
  // indexing scheme must be the same as boundary_vertices
  boundary_faces: Vec<[u16; 3]>,

  params: MeshParams,
}

impl SimMesh {
  fn tetra_val_edges(tetra: [u16; 4], vals: &[Vector3<f32>]) -> Matrix3<f32> {
    Matrix3::from_columns(&[
      vals[tetra[0] as usize] - vals[tetra[3] as usize],
      vals[tetra[1] as usize] - vals[tetra[3] as usize],
      vals[tetra[2] as usize] - vals[tetra[3] as usize],
    ])
  }

  pub fn new(
    (vertex_positions_obj_space, tetras): LoadedMesh,
    params: MeshParams,
  ) -> Self {
    let mut inv_barycentric_3_mat = Vec::new();
    let mut inv_barycentric_4_mat = Vec::new();
    let mut tetra_volume = Vec::new();
    let mut vertex_mass = vec![0.0; vertex_positions_obj_space.len()];

    let mut boundary_faces_set = HashSet::new();

    for tetra in &tetras {
      inv_barycentric_3_mat.push(
        // TODO: fix expect
        Self::tetra_val_edges(*tetra, &vertex_positions_obj_space)
          .try_inverse()
          .expect(
            "all tetrahedrons should have inverses for barycentric coords",
          ),
      );

      inv_barycentric_4_mat.push(
        // TODO: fix expect
        Matrix4::from_columns(&[
          vertex_positions_obj_space[tetra[0] as usize].insert_row(2, 1.0),
          vertex_positions_obj_space[tetra[1] as usize].insert_row(2, 1.0),
          vertex_positions_obj_space[tetra[2] as usize].insert_row(2, 1.0),
          vertex_positions_obj_space[tetra[3] as usize].insert_row(2, 1.0),
        ])
        .try_inverse()
        .expect("all tetrahedrons should have inverses for barycentric coords"),
      );

      let edges = [
        (vertex_positions_obj_space[tetra[1] as usize]
          - vertex_positions_obj_space[tetra[0] as usize]),
        (vertex_positions_obj_space[tetra[2] as usize]
          - vertex_positions_obj_space[tetra[0] as usize]),
        (vertex_positions_obj_space[tetra[3] as usize]
          - vertex_positions_obj_space[tetra[0] as usize]),
      ];

      let volume = (edges[0].cross(&edges[1])).dot(&edges[2]) / 6.0;

      tetra_volume.push(volume);

      for vertex_idx in tetra {
        vertex_mass[*vertex_idx as usize] = params.density * volume / 4.0;
      }

      for face in &mut [
        [tetra[0], tetra[1], tetra[2]],
        [tetra[0], tetra[1], tetra[3]],
        [tetra[0], tetra[2], tetra[3]],
        [tetra[1], tetra[2], tetra[3]],
      ] {
        face.sort();

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
      inv_barycentric_3_mat,
      inv_barycentric_4_mat,
      tetras,
      tetra_volume,
      vertex_mass,
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
    for (((tetra, inv_barycentric_3_mat), inv_barycentric_4_mat), volume) in
      self
        .tetras
        .iter()
        .zip(self.inv_barycentric_3_mat.iter())
        .zip(self.inv_barycentric_4_mat.iter())
        .zip(self.tetra_volume.iter())
    {
      let compute_stress = |vals: &[Vector3<f32>], lambda, mu| {
        let val_edges = Self::tetra_val_edges(*tetra, vals);

        let deformation_grad = val_edges * inv_barycentric_3_mat;

        let strain = 0.5
          * (deformation_grad.transpose() * deformation_grad
            - Matrix3::identity());

        lambda * Matrix3::identity() * strain.trace() + 2.0 * mu * strain
      };

      let elastic_stress = compute_stress(
        positions,
        self.params.incompressibility,
        self.params.rigidity,
      );

      let viscous_stress = compute_stress(
        velocities,
        self.params.viscous_incompressibility,
        self.params.viscous_rigidity,
      );

      let stress = elastic_stress + viscous_stress;

      for (vertex_row, vertex_idx) in tetra.iter().enumerate() {
        let force_unscaled: Vector3<f32> = (0..tetra.len())
          .map(|other_vertex_row| -> Vector3<f32> {
            // TODO: WRONG: FIX BARYCENTRIC/
            let other_vert_col =
              inv_barycentric_4_mat.row(other_vertex_row).remove_column(3);
            let vert_col =
              inv_barycentric_4_mat.row(vertex_row).remove_column(3);
            let sum = (other_vert_col.transpose() * vert_col)
              .component_mul(&stress)
              .sum();

            if cfg!(debug_assertions) {
              let mut check_sum = 0.0;
              for k in 0..2 {
                for l in 0..2 {
                  check_sum +=
                    (inv_barycentric_4_mat.row(other_vertex_row).column(k)
                      * inv_barycentric_4_mat.row(vertex_row).column(l)
                      * stress.row(k).column(l))
                    .sum();
                }
              }

              debug_assert!((check_sum - sum).abs() < 1e-5);
            }

            positions[other_vertex_row] * sum
          })
          .sum();
        let force = (-*volume * 0.5) * force_unscaled;

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
