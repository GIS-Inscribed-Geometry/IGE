//! Synthetic obstacle generators — LER-inspired shapes (walls, strips,
//! corner blocks, diagonal strips) distributed across the polygon interior.
//!
//! Points are exported in two forms: the original `ObstacleInput::Point` for
//! SVG rendering (circles), and a buffered `ObstacleInput::Polygon` for the
//! grid-based solver (zero-area points are inherently grid-invisible).

use geo::algorithm::bounding_rect::BoundingRect;
use geo::Contains;
use geo_types::{Coord, LineString, Point, Polygon};
use ige_core::ObstacleInput;

/// Generates synthetic obstacles and returns two lists:
/// - `solver_obs`: what the solver sees (points buffered to polygons)
/// - `render_obs`: what the SVG renderer draws (original point/lines)
pub fn generate_synth_obs(
    poly: &Polygon<f64>,
    num_points: usize,
    num_lines: usize,
    num_polygons: usize,
) -> (Vec<ObstacleInput>, Vec<ObstacleInput>) {
    let mut solver = Vec::new();
    let mut render = Vec::new();

    let Some(bbox) = poly.bounding_rect() else {
        return (solver, render);
    };
    let min = bbox.min();
    let max = bbox.max();
    let span = (max.x - min.x).max(max.y - min.y);

    // Collect interior grid points, sorted by distance from bbox centre.
    let cx = (min.x + max.x) * 0.5;
    let cy = (min.y + max.y) * 0.5;
    let grid = 50;
    let mut interior: Vec<Coord<f64>> = (0..=grid)
        .flat_map(|si| {
            (0..=grid).filter_map(move |sj| {
                let x = min.x + (max.x - min.x) * si as f64 / grid as f64;
                let y = min.y + (max.y - min.y) * sj as f64 / grid as f64;
                let pt = Point::new(x, y);
                if poly.contains(&pt) {
                    Some(Coord { x, y })
                } else {
                    None
                }
            })
        })
        .collect();
    interior.sort_by(|a, b| {
        let da = (a.x - cx) * (a.x - cx) + (a.y - cy) * (a.y - cy);
        let db = (b.x - cx) * (b.x - cx) + (b.y - cy) * (b.y - cy);
        da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
    });

    let pick = |n: usize, offset: usize| -> Option<&Coord<f64>> {
        if interior.is_empty() {
            return None;
        }
        let idx = if n <= 1 {
            interior.len() / 2
        } else {
            (interior.len() * offset / n).min(interior.len() - 1)
        };
        Some(&interior[idx])
    };

    let half_pt = span * 0.04;

    // -- Points: solver gets a small square; render gets the original point --
    for i in 0..num_points {
        let Some(pt) = pick(num_points.max(1), i) else {
            break;
        };
        // Render as a point (circle in SVG)
        render.push(ObstacleInput::Point(*pt));
        // Solver gets a buffered polygon so it has area
        let h = half_pt;
        solver.push(ObstacleInput::Polygon(Polygon::new(
            LineString::from(vec![
                Coord {
                    x: pt.x - h,
                    y: pt.y - h,
                },
                Coord {
                    x: pt.x + h,
                    y: pt.y - h,
                },
                Coord {
                    x: pt.x + h,
                    y: pt.y + h,
                },
                Coord {
                    x: pt.x - h,
                    y: pt.y + h,
                },
                Coord {
                    x: pt.x - h,
                    y: pt.y - h,
                },
            ]),
            vec![],
        )));
    }

    // -- Lines: proper Line obstacles, not full-span wall polygons --
    let line_len = span * 0.25;
    for i in 0..num_lines {
        let Some(pt) = pick(num_lines.max(1), i) else {
            break;
        };
        let pt2 = pick(num_lines.max(2), (i + 1) % num_lines.max(1)).unwrap_or(pt);
        let dx = pt2.x - pt.x;
        let dy = pt2.y - pt.y;
        let len = (dx * dx + dy * dy).sqrt().max(1e-12);
        let (ux, uy) = (dx / len, dy / len);
        let h = line_len * 0.5;
        let line = LineString::from(vec![
            Coord {
                x: pt.x - ux * h,
                y: pt.y - uy * h,
            },
            Coord {
                x: pt.x + ux * h,
                y: pt.y + uy * h,
            },
        ]);
        solver.push(ObstacleInput::Line(line.clone()));
        render.push(ObstacleInput::Line(line));
    }

    // -- Polygons: truly arbitrary shapes using seeded pseudo-random vertices,
    //    just like generate_polygon_features does for LER testing.
    let block_s = span.min((max.x - min.x).min(max.y - min.y) * 0.15);
    let mut seed = 1u64;
    for i in 0..num_polygons {
        let Some(pt) = pick(num_polygons.max(1), i) else {
            break;
        };
        let s = block_s;
        // Simple xorshift64 for deterministic pseudo-random shapes
        seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let n_sides = 3 + (seed % 6) as usize; // 3..=8 sides
        let mut verts = Vec::with_capacity(n_sides + 1);
        for j in 0..n_sides {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let angle = std::f64::consts::TAU * j as f64 / n_sides as f64;
            // Random radius 0.3..1.0 × half-size produces irregular shapes
            let r = s * 0.5 * (0.3 + (seed as f64 / u64::MAX as f64) * 0.7);
            // Jitter the angle to get non-regular polygons
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let jitter = (seed as f64 / u64::MAX as f64 - 0.5) * std::f64::consts::TAU
                / n_sides as f64
                * 0.5;
            let a = angle + jitter;
            verts.push(Coord {
                x: pt.x + r * a.cos(),
                y: pt.y + r * a.sin(),
            });
        }
        verts.push(verts[0]); // close ring
        let poly = Polygon::new(LineString::from(verts), vec![]);
        solver.push(ObstacleInput::Polygon(poly.clone()));
        render.push(ObstacleInput::Polygon(poly));
    }

    (solver, render)
}
