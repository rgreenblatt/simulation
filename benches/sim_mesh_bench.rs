use criterion::{black_box, criterion_group, criterion_main, Criterion};
use nalgebra::Vector3;
use simulation::load_mesh;
use simulation::ode::{
  EulerSettings, Integrator, IntegratorType, MidpointSettings, RK4Settings,
  SwappableIntegrator,
};
use simulation::simulated_scene::{
  MeshParams, SceneModel, SceneModelParams, SimMesh, S,
};
use std::path::Path;

fn get_mesh_for_repeat(repeat: usize) -> (SimMesh, Vec<Vector3<S>>) {
  let (vertices, tets) = load_mesh(&Path::new("meshes/ellipsoid.mesh"))
    .expect("for benches to be run, the expected mesh files must be present");

  let mut full_vertices = Vec::new();
  let mut full_tet = Vec::new();

  let mut count = 0;

  let total_size = repeat * vertices.len();

  assert!(total_size < std::u16::MAX as usize);

  for _ in 0..repeat {
    full_vertices.extend(&vertices);
    full_tet.extend(tets.iter().map(|tet: &[u16; 4]| {
      let mut new_tet = [0; 4];
      for (tet, new_tet) in tet.iter().zip(&mut new_tet) {
        *new_tet = tet + count;
      }
      new_tet
    }));

    count += vertices.len() as u16;
  }

  let zeros = vec![Vector3::zeros(); total_size];

  let mesh_params = MeshParams {
    incompressibility: 1.0,
    rigidity: 1.0,
    viscous_incompressibility: 1.0,
    viscous_rigidity: 1.0,
    density: 1.0,
  };

  (SimMesh::new((full_vertices, full_tet), mesh_params), zeros)
}

fn vertex_accels(c: &mut Criterion) {
  for repeat_mesh_count in [1, 5, 10, 100].iter() {
    let (mesh, zeros) = get_mesh_for_repeat(*repeat_mesh_count);

    let g = 9.8;

    c.bench_function(
      &format!("vertex accels ellipsoid x {}", repeat_mesh_count),
      |b| {
        b.iter(|| {
          mesh.vertex_accels(
            black_box(&zeros),
            black_box(&zeros),
            black_box(&zeros),
            black_box(g),
          )
        })
      },
    );
  }
}

fn integrator(c: &mut Criterion) {
  for integrator_type in [
    IntegratorType::Euler(EulerSettings {}),
    IntegratorType::Midpoint(MidpointSettings {}),
    IntegratorType::RK4(RK4Settings {}),
  ]
  .iter()
  {
    for repeat_mesh_count in [1, 5, 10, 100].iter() {
      let (mesh, _) = get_mesh_for_repeat(*repeat_mesh_count);

      let g = 9.8;

      let model = SceneModel::new(vec![mesh], SceneModelParams { g }, -100.0);

      let mut integrator = SwappableIntegrator::new(integrator_type.clone());

      let mut state = model.initial_state();

      let mut time = 0.0;
      let time_step = 0.1;

      c.bench_function(
        &format!(
          "integrator {:?} ellipsoid x {}",
          integrator_type, repeat_mesh_count
        ),
        |b| {
          b.iter(|| integrator.step(&model, &mut state, &mut time, &time_step))
        },
      );
    }
  }
}

criterion_group!(benches, vertex_accels, integrator);
criterion_main!(benches);
