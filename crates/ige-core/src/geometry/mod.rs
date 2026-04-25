//! Geometry utilities for LIRiAP

use geo_types::{Coord, LineString, Polygon};

pub fn rotate_polygon(poly: &Polygon<f64>, angle_deg: f64) -> Polygon<f64> {
    if angle_deg.abs() < 1e-9 {
        return poly.clone();
    }
    
    let ext = rotate_coords(&poly.exterior().0, angle_deg);
    let interiors: Vec<_> = poly.interiors().iter()
        .map(|r| rotate_coords(&r.0, angle_deg))
        .collect();
    
    Polygon::new(ext, interiors)
}

fn rotate_coords(coords: &[Coord<f64>], angle_deg: f64) -> LineString<f64> {
    let angle_rad = angle_deg.to_radians();
    let cos_a = angle_rad.cos();
    let sin_a = angle_rad.sin();
    
    let rotated: Vec<_> = coords.iter()
        .map(|c| Coord {
            x: c.x * cos_a - c.y * sin_a,
            y: c.x * sin_a + c.y * cos_a,
        })
        .collect();
    
    LineString::from(rotated)
}