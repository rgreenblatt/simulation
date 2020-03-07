use crate::Scene;
use kiss3d::window::Window;

fn display<S : Scene>(window_name : &str, scene : Scene) {
  let mut window = Window::new(window_name);

  scene.init_objects(&mut window.new_group());
}
