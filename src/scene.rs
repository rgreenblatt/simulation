use kiss3d::scene::SceneNode;
use nalgebra::Point3;

// TODO: have up?
#[derive(Clone,Debug)]
pub struct CameraInfo {
  pub eye: Point3<f32>,
  pub at: Point3<f32>,
}

pub trait SceneGenerator {
  type S: Scene;

  fn init_objects(&self, node: &mut SceneNode) -> Self::S;

  fn default_camera_info(&self) -> CameraInfo;
}

pub trait Scene {
  fn update(&mut self, delta_secs: f32);
}
