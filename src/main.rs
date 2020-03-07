use nalgebra::Point3;
use simulation::{display_scene, CameraInfo, SimulatedSceneGenerator};

fn main() {
  display_scene(
    "simulation",
    &mut SimulatedSceneGenerator::new(CameraInfo {
      eye: Point3::new(5.0, 5.0, 5.0),
      at: Point3::origin(),
    }),
  );
}
