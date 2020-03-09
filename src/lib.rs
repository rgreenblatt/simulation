// #![feature(type_alias_impl_trait)]

pub mod display;
pub mod mesh;
pub mod ode;
pub mod scene;
pub mod simulated_scene;
pub mod utils;

pub use display::display_scene;
pub use mesh::{load_mesh, LoadedMesh};
pub use scene::{CameraInfo, Scene, SceneGenerator};
pub use utils::*;
