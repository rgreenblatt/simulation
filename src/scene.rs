use kiss3d::scene::SceneNode;
use na::Point3;

// TODO: have up?
pub struct CameraInfo {
  pub eye : Point3,
  pub at : Point3,
}

pub trait Scene {
  fn init_objects(&mut self, node: &mut SceneNode);
  
  fn default_camera_info(&self) -> CameraInfo;
  
  fn update(&mut self, delta_secs: f32);
}
