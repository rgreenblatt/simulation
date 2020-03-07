use clap::Clap;
use nalgebra::Point3;
use simulation::{
  display_scene, load_mesh,
  ode::IntegratorType,
  simulated_scene::{
    GlobalParams, IntegrationParams, MeshParams, SceneModelParams,
    SimulatedSceneGenerator, StepParams,
  },
  CameraInfo,
};
use std::path::Path;

#[derive(Clap)]
#[clap(version = "1.0", author = "Ryan G.")]
struct Opts {
  mesh_file: String,
  #[clap(subcommand)]
  integrator: IntegratorType,
}

fn main() -> std::io::Result<()> {
  let opts: Opts = Opts::parse();

  let mesh = load_mesh(&Path::new(&opts.mesh_file))?;

  let mesh_params = MeshParams {
    incompressibility: 2.5,
    rigidity: 2.5,
    viscous_incompressibility: 2.5,
    viscous_rigidity: 2.5,
    density: 1.0,
  };

  display_scene(
    "simulation",
    &mut SimulatedSceneGenerator::new(
      CameraInfo {
        eye: Point3::new(5.0, 5.0, 5.0),
        at: Point3::origin(),
      },
      GlobalParams {
        scene_model_params: SceneModelParams { g: 9.8 },
        integration_params: IntegrationParams {
          step_params: StepParams {
            speed_up: 1.0,
            time_step: 1.0,
          },
          integrator_type: opts.integrator,
        },
      },
      vec![(mesh, mesh_params)],
    ),
  );

  Ok(())
}
