use clap::Clap;
use nalgebra::Point3;
use simulation::{
  display_scene, load_mesh, CameraInfo, SimulatedSceneGenerator,
};
use std::path::Path;

#[derive(Clap)]
#[clap(version = "1.0", author = "Ryan G.")]
struct Opts {
  mesh_file: String,
}

fn main() -> std::io::Result<()> {
  let opts: Opts = Opts::parse();

  let mesh = load_mesh(&Path::new(&opts.mesh_file))?;

  display_scene(
    "simulation",
    &mut SimulatedSceneGenerator::new(CameraInfo {
      eye: Point3::new(5.0, 5.0, 5.0),
      at: Point3::origin(),
    }),
  );

  Ok(())
}
