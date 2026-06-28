pub mod shewchuk;
pub mod winding_index;

use geo_types::{Coord, Polygon};

/// Compute convex hull in CCW order.
pub fn hull_ccw(poly: &Polygon<f64>) -> Result<Vec<Coord<f64>>, ()> {
    // Stub: returns exterior ring
    Ok(poly.exterior().0.clone())
}

/// Compute width and depth at a given angle.
pub fn width_depth(pts: &[Coord<f64>], _angle: f64) -> (f64, f64) {
    if pts.is_empty() {
        return (0.0, 0.0);
    }
    let min_x = pts.iter().map(|c| c.x).fold(f64::INFINITY, f64::min);
    let max_x = pts.iter().map(|c| c.x).fold(f64::NEG_INFINITY, f64::max);
    let min_y = pts.iter().map(|c| c.y).fold(f64::INFINITY, f64::min);
    let max_y = pts.iter().map(|c| c.y).fold(f64::NEG_INFINITY, f64::max);
    (max_x - min_x, max_y - min_y)
}
