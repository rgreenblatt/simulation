#[cfg(test)]
use crate::{assert_float_eq, EPSILON};
use crate::LoadedMesh;
use nalgebra::{Matrix3, Point3, Vector3};
use std::collections::HashSet;
use std::iter::FromIterator;

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
  // not needed?
  vertex_positions_obj_space: Vec<Vector3<f32>>, // per vertex
  vertex_mass: Vec<f32>,                         // per vertex

  tetras: Vec<[u16; 4]>,                    // per tet
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
      get_vertex(1) - get_vertex(0),
      get_vertex(2) - get_vertex(0),
      get_vertex(3) - get_vertex(0),
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

      for (face, opposite_normal) in [
        [tetra[1], tetra[2], tetra[3]],
        [tetra[0], tetra[2], tetra[3]],
        [tetra[0], tetra[1], tetra[3]],
        [tetra[0], tetra[1], tetra[2]],
      ]
      .iter_mut()
      .zip(tet_opposite_normals.iter_mut())
      {
        let get_vertex = |idx| vertex_positions_obj_space[idx as usize];

        let vertices = [
          get_vertex(face[0]),
          get_vertex(face[1]),
          get_vertex(face[2]),
        ];

        *opposite_normal = (vertices[1] - vertices[0])
          .cross(&(vertices[2] - vertices[0]))
          .normalize();

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
      dbg!(deformation_grad);
      let velocity_deformation_grad = compute_deformation_grad(velocities);

      let elastic_strain = 0.5
        * (deformation_grad.transpose() * deformation_grad
          - Matrix3::identity());
      dbg!(elastic_strain);

      // TODO: check
      let viscous_strain = 0.5
        * (deformation_grad.transpose() * velocity_deformation_grad
          + velocity_deformation_grad.transpose() * deformation_grad);

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
        let force = mat
          * self.vertex_positions_obj_space[*vertex_idx as usize]
            .component_mul(opposite_normal);

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

#[test]
fn test_1_tet_sim_mesh() {
  let positions = vec![
    Vector3::new(0.0, 0.0, 0.0),
    Vector3::new(1.0, 0.0, 0.0),
    Vector3::new(0.0, 1.0, 0.0),
    Vector3::new(0.0, 0.0, 1.0),
  ];
  let tetras = vec![[0, 1, 2, 3]];
  let density = 1.0;
  let params = MeshParams {
    incompressibility: 0.0,
    rigidity: 0.0,
    viscous_incompressibility: 0.0,
    viscous_rigidity: 0.0,
    density,
  };

  let mesh = SimMesh::new((positions, tetras.clone()), params.clone());

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
  let total_mass: f32 = vol * density;
  let actual_total_mass: f32 = mesh.vertex_mass.iter().sum();
  assert_float_eq!(total_mass, actual_total_mass);
  let each_mass = actual_total_mass / 4.0;
  for mass in &mesh.vertex_mass {
    assert_float_eq!(mass, each_mass);
  }
}

#[test]
fn test_2_tet_sim_mesh() {
  let positions = vec![
    Vector3::new(0.0, 0.0, 0.0),
    Vector3::new(1.0, 0.0, 0.0),
    Vector3::new(0.0, 1.0, 0.0),
    Vector3::new(0.0, 0.0, 1.0),
    Vector3::new(1.0, 1.0, 0.0),
  ];
  let tetras = vec![[0, 1, 2, 3], [4, 1, 2, 3]];
  let density = 1.0;
  let params = MeshParams {
    incompressibility: 0.0,
    rigidity: 0.0,
    viscous_incompressibility: 0.0,
    viscous_rigidity: 0.0,
    density,
  };

  let mesh = SimMesh::new((positions, tetras.clone()), params.clone());

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
  let total_mass: f32 = vol * density;
  let actual_total_mass: f32 = mesh.vertex_mass.iter().sum();
  assert_float_eq!(total_mass, actual_total_mass);
  assert_float_eq!(mesh.vertex_mass[0], total_mass / 2.0 / 4.0);
  assert_float_eq!(mesh.vertex_mass[4], total_mass / 2.0 / 4.0);
  let each_mass = total_mass / 4.0;
  for mass in &mesh.vertex_mass[1..4] {
    assert_float_eq!(mass, each_mass);
  }
}

#[test]
fn test_just_gravity_sim_mesh() {
  let density = 0.0381;
  let params = MeshParams {
    incompressibility: 0.0,
    rigidity: 0.0,
    viscous_incompressibility: 0.0,
    viscous_rigidity: 0.0,
    density,
  };

  let check = |positions: &Vec<_>, tetras: &Vec<_>, g| {
    let mesh =
      SimMesh::new((positions.clone(), tetras.clone()), params.clone());
    let accels = mesh.vertex_accels(
      &positions,
      &vec![Vector3::new(0.0, 0.0, 0.0); positions.len()],
      &vec![Vector3::new(0.0, 0.0, 0.0); positions.len()],
      g,
    );
    for accel in accels {
      assert_float_eq!(accel[0], 0.0);
      assert_float_eq!(accel[1], -g);
      assert_float_eq!(accel[2], 0.0);
    }
  };

  let positions = vec![
    Vector3::new(0.0, 0.0, 0.0),
    Vector3::new(1.0, 0.0, 0.0),
    Vector3::new(0.0, 1.0, 0.0),
    Vector3::new(0.0, 0.0, 1.0),
  ];
  let tetras = vec![[0, 1, 2, 3]];

  check(&positions, &tetras, 0.7);
  check(&positions, &tetras, 0.1);
  check(&positions, &tetras, 18.0);

  let positions = vec![
    Vector3::new(0.0, 0.0, 0.0),
    Vector3::new(1.0, 0.0, 0.0),
    Vector3::new(0.0, 1.0, 0.0),
    Vector3::new(0.0, 0.0, 1.0),
    Vector3::new(1.0, 1.0, 0.0),
  ];
  let tetras = vec![[0, 1, 2, 3], [4, 1, 2, 3]];

  check(&positions, &tetras, 0.7);
  check(&positions, &tetras, 0.1);
  check(&positions, &tetras, 18.0);
}

#[test]
fn test_just_compressibility_sim_mesh() {
  let density = 1.0;
  let params = MeshParams {
    incompressibility: 0.0,
    rigidity: 1.0,
    viscous_incompressibility: 0.0,
    viscous_rigidity: 0.0,
    density,
  };

  let positions = vec![
    Vector3::new(0.0, 0.0, 0.0),
    Vector3::new(1.0, 0.0, 0.0),
    Vector3::new(0.0, 1.0, 0.0),
    Vector3::new(0.0, 0.0, 1.0),
  ];
  let tetras = vec![[0, 1, 2, 3]];

  let mesh = SimMesh::new((positions.clone(), tetras.clone()), params.clone());
  let accels = mesh.vertex_accels(
    &positions,
    &vec![Vector3::new(0.0, 0.0, 0.0); positions.len()],
    &vec![Vector3::new(0.0, 0.0, 0.0); positions.len()],
    0.0,
  );
  for accel in accels {
    assert_float_eq!(accel[0], 0.0);
    assert_float_eq!(accel[1], 0.0);
    assert_float_eq!(accel[2], 0.0);
  }
  
  let new_positions = vec![
    Vector3::new(0.0, 0.0, 0.0),
    Vector3::new(1.0, 0.0, 0.0),
    Vector3::new(0.0, 1.5, 0.0),
    Vector3::new(0.0, 0.0, 1.0),
  ];
  
  let accels = mesh.vertex_accels(
    &new_positions,
    &vec![Vector3::new(0.0, 0.0, 0.0); positions.len()],
    &vec![Vector3::new(0.0, 0.0, 0.0); positions.len()],
    0.0,
  );
    
  dbg!(&accels);

  for accel in accels {
    dbg!(accel);
    assert_float_eq!(accel[0], 0.0);
    assert!(accel[1] <= EPSILON);
    assert_float_eq!(accel[2], 0.0);
  }
}
