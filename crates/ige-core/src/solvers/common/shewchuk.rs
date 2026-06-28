/// Adaptive-precision orientation test using Shewchuk's algorithm.
/// Returns > 0 if (ax,ay), (bx,by), (cx,cy) are CCW,
///         < 0 if CW,
///         = 0 if collinear.
#[inline]
pub fn orient2d(ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64) -> f64 {
    (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
}
