pub const EPSILON : f32 = 1e-5;

#[macro_export]
macro_rules! assert_float_eq {
  ($l : expr, $r : expr) => {
    assert!(($l - $r).abs() < 1e-5);
  };
}
