use crate::{
  ode::{Integrator, IntegratorType, SwappableIntegrator},
  simulated_scene::{
    MeshParams, SceneModel, SceneModelParams, SceneModelState, SimMesh,
  },
  CameraInfo, LoadedMesh, Scene, SceneGenerator,
};
use kiss3d::resource::Mesh as Kiss3dMesh;
use kiss3d::scene::SceneNode;
use nalgebra::{Point3, Translation3, Vector3};
use std::cell::RefCell;
use std::rc::Rc;

#[derive(Clone, Debug)]
pub struct StepParams {
  pub time_step: f32,
  pub speed_up: f32,
}

#[derive(Clone, Debug)]
pub struct IntegrationParams {
  pub integrator_type: IntegratorType,
  pub step_params: StepParams,
}

#[derive(Clone, Debug)]
pub struct GlobalParams {
  pub scene_model_params: SceneModelParams,
  pub integration_params: IntegrationParams,
}

// TODO: make configurable
pub struct SimulatedSceneGenerator {
  camera_info: CameraInfo,
  integration_params: IntegrationParams,
  scene_model: SceneModel,
}

impl SimulatedSceneGenerator {
  pub fn new(
    camera_info: CameraInfo,
    global_params: GlobalParams,
    meshs: Vec<(LoadedMesh, MeshParams)>,
  ) -> Self {
    let GlobalParams {
      scene_model_params,
      integration_params,
    } = global_params;

    Self {
      camera_info,
      integration_params,
      scene_model: SceneModel::new(
        meshs
          .into_iter()
          .map(|(mesh, params)| SimMesh::new(mesh, params))
          .collect(),
        scene_model_params,
        -3.0, // TODO: input
      ),
    }
  }
}

pub struct SimulatedScene {
  meshes: Vec<Rc<RefCell<Kiss3dMesh>>>,
  scene_model: SceneModel,
  scene_state: SceneModelState,
  step_params: StepParams,
  integrator: SwappableIntegrator<SceneModel>,
}

impl Scene for SimulatedScene {
  fn update(&mut self, delta_secs: f32) {
    let steps = (delta_secs / self.step_params.time_step).ceil() as usize;

    self.integrator.n_steps(
      &self.scene_model,
      &mut self.scene_state,
      &mut 0.0,
      &(delta_secs / steps as f32),
      steps,
    );

    for ((mesh, sim_mesh), mesh_interval) in self
      .meshes
      .iter()
      .zip(self.scene_model.meshs().iter())
      .zip(self.scene_model.mesh_intervals().iter())
    {
      let (positions, faces) = sim_mesh.boundary_vertices_faces(
        &self.scene_state.positions
          [(mesh_interval[0] as usize)..(mesh_interval[1] as usize)],
      );

      mesh.replace(Kiss3dMesh::new(positions, faces, None, None, true));
    }
  }
}

impl SceneGenerator for SimulatedSceneGenerator {
  type S = SimulatedScene;

  fn init_objects(&self, node: &mut SceneNode) -> Self::S {
    let floor_thickness = 0.2;
    let floor_width = 5.0;
    let floor_depth = 5.0;
    let mut floor_node =
      node.add_cube(floor_width, floor_thickness, floor_depth);
    floor_node.append_translation(&Translation3::new(
      0.0,
      (-0.5 * floor_thickness) + self.scene_model.floor_height(),
      0.0,
    ));
    floor_node.set_color(0.0, 0.0, 1.0);

    SimulatedScene {
      meshes: (0..self.scene_model.meshs().len())
        .map(|_| {
          let mesh = Rc::new(RefCell::new(Kiss3dMesh::new(
            Vec::new(),
            Vec::new(),
            None,
            None,
            true,
          )));

          let mut mesh_scene_node =
            node.add_mesh(mesh.clone(), Vector3::new(1.0, 1.0, 1.0));

          mesh_scene_node.enable_backface_culling(false);
          mesh_scene_node.set_color(1.0, 0.0, 0.0);
          mesh_scene_node.set_lines_color(Some(Point3::new(0.0, 0.0, 0.0)));
          mesh_scene_node.set_lines_width(3.0);

          mesh
        })
        .collect(),
      scene_model: self.scene_model.clone(),
      scene_state: self.scene_model.initial_state(),
      step_params: self.integration_params.step_params.clone(),
      integrator: SwappableIntegrator::new(
        self.integration_params.integrator_type.clone(),
      ),
    }
  }

  fn default_camera_info(&self) -> CameraInfo {
    self.camera_info.clone()
  }
}
