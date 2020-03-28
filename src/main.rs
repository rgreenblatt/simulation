use clap::Clap;
use nalgebra::{Point3, Rotation3, Transform3, Vector3};
use simulation::{
  display_scene, load_mesh_with_transform,
  ode::IntegratorType,
  simulated_scene::S as Scalar,
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

  #[clap(long = "speed-up", default_value = "1.0")]
  speed_up: Scalar,

  #[clap(short = "t", long = "time-step", default_value = "0.0025")]
  time_step: Scalar,

  #[clap(short = "l", long = "incompressibility", default_value = "100.0")]
  /// lambda
  incompressibility: Scalar,

  #[clap(short = "m", long = "rigidity", default_value = "100.0")]
  /// mu
  rigidity: Scalar,

  #[clap(
    short = "p",
    long = "viscous_incompressibility",
    default_value = "2.0"
  )]
  /// phi
  viscous_incompressibility: Scalar,

  #[clap(short = "s", long = "viscous_rigidity", default_value = "5.0")]
  /// psi
  viscous_rigidity: Scalar,

  #[clap(short = "d", long = "density", default_value = "5.0")]
  density: Scalar,

  #[clap(
    short = "g",
    long = "gravity",
    default_value = "9.8",
    allow_hyphen_values = true
  )]
  g: Scalar,

  #[clap(long = "penalty-force", default_value = "10000.0")]
  penalty_force: Scalar,

  #[clap(long = "floor-friction-coeff", default_value = "0.1")]
  floor_friction_coeff: Scalar,

  #[clap(long = "sphere-radius", default_value = "1.0")]
  sphere_radius: Scalar,

  #[clap(
    long = "sphere-pos-x",
    default_value = "0.0",
    allow_hyphen_values = true
  )]
  sphere_pos_x: Scalar,

  #[clap(
    long = "sphere-pos-y",
    default_value = "-3.5",
    allow_hyphen_values = true
  )]
  sphere_pos_y: Scalar,

  #[clap(
    long = "sphere-pos-z",
    default_value = "0.0",
    allow_hyphen_values = true
  )]
  sphere_pos_z: Scalar,

  #[clap(
    long = "floor-pos",
    default_value = "-3.0",
    allow_hyphen_values = true
  )]
  floor_pos: Scalar,

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
    g,
    penalty_force,
    floor_friction_coeff,
    sphere_radius,
    sphere_pos_x,
    sphere_pos_y,
    sphere_pos_z,
    floor_pos,
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
        scene_model_params: SceneModelParams { g },
        integration_params: IntegrationParams {
          step_params: StepParams {
            speed_up,
            time_step,
          },
          integrator_type,
        },
      },
      vec![(mesh, mesh_params)],
      penalty_force,
      floor_friction_coeff,
      floor_pos,
      sphere_radius,
      Vector3::new(sphere_pos_x, sphere_pos_y, sphere_pos_z),
    ),
  )?;

  Ok(())
}
