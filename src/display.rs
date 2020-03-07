use crate::{CameraInfo, Scene, SceneGenerator};
use kiss3d::camera::FirstPerson;
use kiss3d::light::Light;
use kiss3d::window::Window;
use std::time::Instant;

pub fn display_scene<S: SceneGenerator>(window_name: &str, scene_gen: &mut S) {
  let mut window = Window::new(window_name);

  let mut scene = scene_gen.init_objects(&mut window.add_group());

  window.set_light(Light::StickToCamera);

  let mut time_since_last = Instant::now();

  let CameraInfo { eye, at } = scene_gen.default_camera_info();

  let mut cam = FirstPerson::new(eye, at);

  while window.render_with_camera(&mut cam) {
    scene.update(time_since_last.elapsed().as_secs_f32());

    time_since_last = Instant::now();
  }
}
