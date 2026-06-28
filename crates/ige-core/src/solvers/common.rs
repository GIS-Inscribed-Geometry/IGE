//! Common utilities for shewchuk-adaptive-precision solver features.

use geo_types::{Coord, LineString};
use robust::orient2d;

/// Compute the signed area of a ring using adaptive precision.
pub fn ring_area(ring: &LineString<f64>) -> f64 {
    let mut area = 0.0;
    let coords = &ring.0;
    for i in 0..coords.len().saturating_sub(1) {
        area += orient2d(coords[i], coords[i + 1], coords[0]);
    }
    area * 0.5
}

/// Check if a ring is oriented clockwise.
pub fn is_cw(ring: &LineString<f64>) -> bool {
    ring_area(ring) < 0.0
}

/// Check if a ring is oriented counter-clockwise.
pub fn is_ccw(ring: &LineString<f64>) -> bool {
    ring_area(ring) > 0.0
}
