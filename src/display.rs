use crate::{CameraInfo, Scene, SceneGenerator};
use indicatif::ProgressBar;
use kiss3d::camera::FirstPerson;
use kiss3d::light::Light;
use kiss3d::window::Window;
use std::fs::create_dir_all;
use std::io;
use std::path::Path;
use std::time::Instant;

pub fn display_scene<S: SceneGenerator>(
  window_name: &str,
  hide: bool,
  record_image_dir: Option<&Path>,
  frame_limit: Option<usize>,
  force_sim_fps: Option<f32>,
  scene_gen: &mut S,
) -> io::Result<()> {
  if let Some(record_image_dir) = record_image_dir {
    if record_image_dir.exists() {
      if !record_image_dir.is_dir() {
        eprintln!("Record image directory exists and isn't directory, exiting");
        return Ok(());
      }
    } else {
      create_dir_all(record_image_dir)?;
    }
  }

  let mut window = Window::new_hidden(window_name);

  if !hide {
    window.show();
  }

  let mut scene = scene_gen.init_objects(&mut window.add_group());

  window.set_light(Light::StickToCamera);

  let mut time_since_last = Instant::now();

  let CameraInfo { eye, at } = scene_gen.default_camera_info();

  let mut iters = 0;

  let mut cam = FirstPerson::new(eye, at);

  let mut frame_limit_bar = frame_limit
    .map(|frame_limit| (frame_limit, ProgressBar::new(frame_limit as u64)));

  while window.render_with_camera(&mut cam) {
    let delta_time = force_sim_fps
      .map(|fps| 1.0 / fps)
      .unwrap_or_else(|| time_since_last.elapsed().as_secs_f32());
    time_since_last = Instant::now();

    if let Some(record_image_dir) = record_image_dir {
      window
        .snap_image()
        .save(record_image_dir.join(format!("output_{}.png", iters)))?;
    }

    iters += 1;

    if let Some((frame_limit, p_bar)) = &mut frame_limit_bar {
      if iters >= *frame_limit {
        break;
      }
      p_bar.inc(1);
    }

    scene.update(delta_time);
  }

  if let Some((_, p_bar)) = &mut frame_limit_bar {
    p_bar.finish();
  }

  Ok(())
}
