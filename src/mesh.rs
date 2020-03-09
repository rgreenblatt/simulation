use nalgebra::{Transform3, Vector3};
use regex::Regex;
use std::fs::File;
use std::io::{prelude::*, BufReader};
use std::path::Path;

pub type LoadedMesh = (Vec<Vector3<f32>>, Vec<[u16; 4]>);

pub fn load_mesh_with_transform(
  path: &Path,
  transform: Option<&Transform3<f32>>,
) -> std::io::Result<LoadedMesh> {
  let reader = BufReader::new(File::open(path)?);

  let vertex_re =
    Regex::new(r"v (-?\d*\.?\d+) +(-?\d*\.?\d+) +(-?\d*\.?\d+)").unwrap();
  let tetra_re = Regex::new(r"t (\d+) +(\d+) +(\d+) +(\d+)").unwrap();

  let mut vertices = Vec::new();
  let mut tetras = Vec::new();

  for line in reader.lines() {
    let line = line?;
    if let Some(matchs) = vertex_re.captures(&line) {
      debug_assert_eq!(matchs.len(), 4);

      let iter = matchs
        .iter()
        .skip(1)
        .map(|v| v.unwrap().as_str().parse().unwrap());

      let mut vert = Vector3::zeros();

      for (i, val) in iter.enumerate() {
        vert[i] = val;
      }

      if let Some(transform) = transform {
        vert = transform * vert;
      }

      vertices.push(vert);
    } else if let Some(matchs) = tetra_re.captures(&line) {
      debug_assert_eq!(matchs.len(), 5);

      let iter = matchs
        .iter()
        .skip(1)
        .map(|v| v.unwrap().as_str().parse().unwrap());

      let mut tetra = [0; 4];

      for (i, val) in iter.enumerate() {
        tetra[i] = val;
      }

      tetras.push(tetra);
    }
  }

  Ok((vertices, tetras))
}

pub fn load_mesh(path: &Path) -> std::io::Result<LoadedMesh> {
  load_mesh_with_transform(path, None)
}
