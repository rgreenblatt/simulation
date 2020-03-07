use crate::{CameraInfo, Scene};
use kiss3d::camera::FirstPerson;
use kiss3d::light::Light;
use kiss3d::window::Window;
use std::time::Instant;

fn display<S: Scene>(window_name: &str, scene: &mut S) {
  let mut window = Window::new(window_name);

  scene.init_objects(&mut window.add_group());

  window.set_light(Light::StickToCamera);

  let mut time_since_last = Instant::now();

  let CameraInfo { eye, at } = scene.default_camera_info();

  let mut cam = FirstPerson::new(eye, at);

  while window.render_with_camera(&mut cam) {
    scene.update(time_since_last.elapsed().as_secs_f32());

    time_since_last = Instant::now();
  }
}
