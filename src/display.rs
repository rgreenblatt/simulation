use crate::Scene;
use kiss3d::window::Window;

fn display<S : Scene>(window_name : &str, scene : &mut S) {
  let mut window = Window::new(window_name);

  scene.init_objects(&mut window.add_group());
}
