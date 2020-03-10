use clap::Clap;
use nalgebra::{Point3, Rotation3, Transform3, Vector3};
use simulation::{
  display_scene, load_mesh_with_transform,
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

  #[clap(short = "h", long = "hide")]
  hide: bool,

  #[clap(short = "r", long = "record-image-dir")]
  record_image_dir: Option<String>,

  #[clap(short = "f", long = "frame-limit")]
  frame_limit: Option<usize>,

  #[clap(long = "force-sim-fps")]
  force_sim_fps: Option<f32>,

  #[clap(long = "speed_up", default_value = "1.0")]
  speed_up: f32,

  #[clap(short = "t", long = "time-step", default_value = "0.0005")]
  time_step: f32,

  #[clap(short = "l", long = "incompressibility", default_value = "100.0")]
  /// lambda
  incompressibility: f32,

  #[clap(short = "m", long = "rigidity", default_value = "100.0")]
  /// mu
  rigidity: f32,

  #[clap(
    short = "p",
    long = "viscous_incompressibility",
    default_value = "10.0"
  )]
  /// phi
  viscous_incompressibility: f32,

  #[clap(short = "s", long = "viscous_rigidity", default_value = "10.0")]
  /// psi
  viscous_rigidity: f32,

  #[clap(short = "d", long = "density", default_value = "5.0")]
  density: f32,

  #[clap(subcommand)]
  integrator_type: IntegratorType,
}

fn main() -> std::io::Result<()> {
  let Opts {
    mesh_file,
    hide,
    record_image_dir,
    frame_limit,
    force_sim_fps,
    speed_up,
    time_step,
    incompressibility,
    rigidity,
    viscous_rigidity,
    viscous_incompressibility,
    density,
    integrator_type,
  } = Opts::parse();

  let mesh = load_mesh_with_transform(
    &Path::new(&mesh_file),
    Some(&Transform3::from_matrix_unchecked(
      Rotation3::new(Vector3::new(0.0, 1.0, 0.0)).to_homogeneous(),
    )),
  )?;

  let mesh_params = MeshParams {
    incompressibility,
    rigidity,
    viscous_incompressibility,
    viscous_rigidity,
    density,
  };

  display_scene(
    "simulation",
    hide,
    record_image_dir.as_ref().map(|v| Path::new(v)),
    frame_limit,
    force_sim_fps,
    &mut SimulatedSceneGenerator::new(
      CameraInfo {
        eye: Point3::new(5.0, 0.0, 5.0),
        at: Point3::new(0.0, -3.0, 0.0),
      },
      GlobalParams {
        scene_model_params: SceneModelParams { g: 9.8 },
        integration_params: IntegrationParams {
          step_params: StepParams {
            speed_up,
            time_step,
          },
          integrator_type,
        },
      },
      vec![(mesh, mesh_params)],
    ),
  )?;

  Ok(())
}
