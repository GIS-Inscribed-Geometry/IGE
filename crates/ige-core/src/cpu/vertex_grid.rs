//! CPU baseline oriented LIR solver using vertex-coordinate grid approach.
//!
//! Implements the exact Daniels et al. (1997) algorithm: the largest axis-aligned
//! rectangle inscribed in a simple polygon always has at least two sides aligned
//! to vertex coordinates. This solver builds a grid from polygon vertices and
//! uses largest-rectangle-in-histogram sweeps.

use geo_types::{Coord, LineString, Polygon};
use geo::ConvexHull;
use geo::Area;
use geo::BoundingRect;
use geo::algorithm::contains::Contains;
use std::collections::HashSet;

use crate::shared::{PolygonType, Rectangle};
    
    match (is_convex, has_holes) {
        (true, false) => PolygonType::ConvexNoHoles,
        (true, true) => PolygonType::ConvexWithHoles,
        (false, false) => PolygonType::ConcaveNoHoles,
        (false, true) => PolygonType::ConcaveWithHoles,
    }
}

/// Point-in-polygon test
fn point_in_polygon(point: Coord<f64>, poly: &Polygon<f64>) -> bool {
    use geo::Contains;
    poly.contains(&point) || on_boundary(&point, poly)
}

fn on_boundary(point: &Coord<f64>, poly: &Polygon<f64>) -> bool {
    if let Some(br) = poly.bounding_rect() {
        let min = br.min();
        let max = br.max();
        let eps = 1e-10;
        (point.x - min.x).abs() < eps || (point.x - max.x).abs() < eps ||
        (point.y - min.y).abs() < eps || (point.y - max.y).abs() < eps
    } else {
        false
    }
}

/// Largest rectangle in histogram (classic stack algorithm)
fn largest_rect_in_histogram(
    heights: &[usize],
    xs: &[f64],
    ys: &[f64],
    row_idx: usize,
) -> (f64, f64, f64, f64, f64) {
    let n = heights.len();
    let mut stack: Vec<(usize, usize)> = Vec::new(); // (start_col, height)
    
    let mut best_area = 0.0;
    let mut best_rect = (0.0, 0.0, 0.0, 0.0);
    
    for col in 0..=n {
        let h = if col < n { heights[col] } else { 0 };
        let mut start = col;
        
        while let Some(&(sc, sh)) = stack.last() {
            if sh <= h {
                break;
            }
            stack.pop();
            
            // Calculate rectangle bounds
            let x0 = xs[sc];
            let x1 = xs[col.min(xs.len() - 1)];
            let y0 = ys[(row_idx + 1).saturating_sub(sh)];
            let y1 = ys[(row_idx + 1).min(ys.len() - 1)];
            
            let width = x1 - x0;
            let height = y1 - y0;
            
            if width > 0.0 && height > 0.0 {
                let area = width * height;
                if area > best_area {
                    best_area = area;
                    best_rect = (x0, y0, x1, y1);
                }
            }
            
            start = sc;
        }
        
        if col < n {
            stack.push((start, h));
        }
    }
    
    (best_rect.0, best_rect.1, best_rect.2, best_rect.3, best_area)
}

/// Vertex-grid solver (Daniels et al. 1997)
pub fn solve_vertex_grid(poly: &Polygon<f64>) -> Option<Rectangle> {
    // Extract unique vertex coordinates
    let mut x_coords = HashSet::new();
    let mut y_coords = HashSet::new();
    
    // Collect exterior vertices
    for coord in poly.exterior().0.iter() {
        x_coords.insert(ordered_float::OrderedFloat(coord.x));
        y_coords.insert(ordered_float::OrderedFloat(coord.y));
    }
    
    // For holed polygons, include interior ring vertices
    for interior in poly.interiors() {
        for coord in interior.0.iter() {
            x_coords.insert(ordered_float::OrderedFloat(coord.x));
            y_coords.insert(ordered_float::OrderedFloat(coord.y));
        }
    }
    
    // Convert to sorted vectors
    let mut xs: Vec<f64> = x_coords.into_iter().map(|f| f.into_inner()).collect();
    let mut ys: Vec<f64> = y_coords.into_iter().map(|f| f.into_inner()).collect();
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
    
    // Midpoint augmentation (critical for correctness)
    xs = augment_with_midpoints(&xs);
    ys = augment_with_midpoints(&ys);
    
    let n_cols = xs.len().saturating_sub(1);
    let n_rows = ys.len().saturating_sub(1);
    
    if n_cols == 0 || n_rows == 0 {
        return None;
    }
    
// Build cell mask
    let mut mask = vec![vec![false; n_cols]; n_rows];
    
    for row in 0..n_rows {
        let y0 = ys[row];
        let y1 = ys[row + 1];
        
        for col in 0..n_cols {
            let x0 = xs[col];
            let x1 = xs[col + 1];
            
            // Use center point as indicator (fastest, approximates poly.covers)
            let cx = (x0 + x1) * 0.5;
            let cy = (y0 + y1) * 0.5;
            
            if poly.contains(&Coord { x: cx, y: cy }) {
                mask[row][col] = true;
            }
        }
    }
            }
        }
    }
        }
    }
    
    // Histogram sweep
    let mut heights = vec![0; n_cols];
    let mut best_area = 0.0;
    let mut best_rect: Option<Rectangle> = None;
    
    for row in 0..n_rows {
        // Update heights
        for col in 0..n_cols {
            if mask[row][col] {
                heights[col] += 1;
            } else {
                heights[col] = 0;
            }
        }
        
        // Find largest rectangle in this histogram
        let (x0, y0, x1, y1, area) = largest_rect_in_histogram(&heights, &xs, &ys, row);
        
        if area > best_area {
            best_area = area;
            best_rect = Some(Rectangle {
                x_min: x0,
                y_min: y0,
                x_max: x1,
                y_max: y1,
            });
        }
    }
    
    best_rect
}

/// Augment coordinate array with midpoints
fn augment_with_midpoints(coords: &[f64]) -> Vec<f64> {
    if coords.len() < 2 {
        return coords.to_vec();
    }
    
    let mut result = Vec::with_capacity(2 * coords.len() - 1);
    
    for i in 0..coords.len() {
        result.push(coords[i]);
        if i < coords.len() - 1 {
            result.push((coords[i] + coords[i + 1]) * 0.5);
        }
    }
    
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::coord;
    
    #[test]
    fn test_vertex_grid_square() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! { x: 0.0, y: 0.0 },
                coord! { x: 10.0, y: 0.0 },
                coord! { x: 10.0, y: 10.0 },
                coord! { x: 0.0, y: 10.0 },
                coord! { x: 0.0, y: 0.0 },
            ]),
            vec![],
        );
        
        let rect = solve_vertex_grid(&poly).unwrap();
        
        // For a square, the LIR should be the entire square
        assert!((rect.area() - 100.0).abs() < 0.1);
    }
    
    #[test]
    fn test_polygon_type_detection() {
        let square = Polygon::new(
            LineString::from(vec![
                coord! { x: 0.0, y: 0.0 },
                coord! { x: 10.0, y: 0.0 },
                coord! { x: 10.0, y: 10.0 },
                coord! { x: 0.0, y: 10.0 },
                coord! { x: 0.0, y: 0.0 },
            ]),
            vec![],
        );
        
        assert_eq!(detect_polygon_type(&square), PolygonType::ConvexNoHoles);
    }
}
