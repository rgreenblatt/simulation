use crate::{CameraInfo, Scene, SceneGenerator};
use kiss3d::scene::SceneNode;
use nalgebra::geometry::Rotation3;
use nalgebra::Vector3;

// TODO: make configurable
pub struct SimulatedSceneGenerator {
  camera_info: CameraInfo,
}

impl SimulatedSceneGenerator {
  pub fn new(camera_info: CameraInfo) -> Self {
    Self { camera_info }
  }
}

pub struct SimulatedScene {
  cube: SceneNode,
}

impl Scene for SimulatedScene {
  fn update(&mut self, delta_secs: f32) {
    self.cube.prepend_to_local_rotation(
      &Rotation3::new(Vector3::new(1.0, 1.0, 0.0) * delta_secs).into(),
    );

    // mesh.replace(Mesh::new(
    //   base_vertices
    //     .iter()
    //     .map(|v| Point3::from(v.coords + addr))
    //     .collect(),
    //   indices.clone(),
    //   None,
    //   None,
    //   true,
    // ));
  }
}

impl SceneGenerator for SimulatedSceneGenerator {
  type S = SimulatedScene;

  fn init_objects(&self, node: &mut SceneNode) -> Self::S {
    let mut cube = node.add_cube(2.0, 1.0, 1.0);

    cube.set_color(1.0, 0.0, 0.0);
    cube.enable_backface_culling(true);

    // let mesh = Rc::new(RefCell::new(Mesh::new(
    //   Vec::new(),
    //   Vec::new(),
    //   None,
    //   None,
    //   true,
    // )));
    // let set = mesh.clone();

    SimulatedScene { cube }
  }

  fn default_camera_info(&self) -> CameraInfo {
    self.camera_info.clone()
  }
}
