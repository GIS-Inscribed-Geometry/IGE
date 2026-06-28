use geo::{Area, BoundingRect, Centroid};
use geo_types::{Coord, LineString, Point, Polygon};
use rayon::prelude::*;

use crate::shared::{LirError, ObstacleInput, Rectangle, Result};
use crate::solvers::ler::oriented::mask::{build_free_mask, rotated_poly_bbox, LineSegRot};
use crate::solvers::lir::axis_aligned::histogram::{lrih, lrih_vp};
use crate::solvers::lir::oriented::candidates::edge_candidate_angles;
use crate::solvers::lir::oriented::certify::certify_and_adjust;
use crate::solvers::lir::oriented::expand::expand_rect_to_boundary;
use crate::solvers::lir::oriented::parallel::{
    build_mask_parallel, rotate_coords_only, RotatedCoords,
};

use super::combined::{buffer_lines_to_polygons, group_obstacles, LineObs};
use super::{LirObstaclesOptions, LirObstaclesResult};

const EPS: f64 = 1e-9;

#[derive(Debug, Clone, Copy)]
struct ObsCandidate {
    angle: f64,
    area: f64,
    rect_rot: (f64, f64, f64, f64),
}

fn generate_angles(poly: &Polygon<f64>, min_angles: usize) -> Vec<f64> {
    let mut angles = edge_candidate_angles(poly, 4.0, 12);
    if angles.len() < min_angles {
        let step = 5usize.max(1);
        for step_deg in (step..90).step_by(step) {
            let a = step_deg as f64;
            if !angles.iter().any(|&ea| (ea - a).abs() < 0.5) {
                angles.push(a);
            }
        }
    }
    angles.sort_by(|a, b| a.partial_cmp(b).unwrap());
    angles.dedup_by(|a, b| (*a - *b).abs() < 0.1);
    angles
}

fn rotate_point(x: f64, y: f64, angle_deg: f64, origin: &Point<f64>) -> Coord<f64> {
    let rad = angle_deg.to_radians();
    let cos_a = rad.cos();
    let sin_a = rad.sin();
    let dx = x - origin.x();
    let dy = y - origin.y();
    Coord {
        x: origin.x() + dx * cos_a - dy * sin_a,
        y: origin.y() + dx * sin_a + dy * cos_a,
    }
}

fn rotate_points(pts: &[Coord<f64>], centroid: Point<f64>, angle_deg: f64) -> Vec<Coord<f64>> {
    let (cx, cy) = (centroid.x(), centroid.y());
    let rad = -angle_deg.to_radians();
    let (cos_a, sin_a) = (rad.cos(), rad.sin());
    pts.iter()
        .map(|c| Coord {
            x: cx + (c.x - cx) * cos_a - (c.y - cy) * sin_a,
            y: cy + (c.x - cx) * sin_a + (c.y - cy) * cos_a,
        })
        .collect()
}

fn rotate_point_coord(c: &Coord<f64>, centroid: Point<f64>, angle_deg: f64) -> (f64, f64) {
    let (cx, cy) = (centroid.x(), centroid.y());
    let rad = -angle_deg.to_radians();
    let (cos_a, sin_a) = (rad.cos(), rad.sin());
    let dx = c.x - cx;
    let dy = c.y - cy;
    (cx + dx * cos_a - dy * sin_a, cy + dx * sin_a + dy * cos_a)
}

fn extract_obstacle_points(polygons: &[Polygon<f64>]) -> Vec<Coord<f64>> {
    let mut pts = Vec::new();
    for p in polygons {
        for c in p.exterior().coords() {
            pts.push(*c);
        }
        for hole in p.interiors() {
            for c in hole.coords() {
                pts.push(*c);
            }
        }
    }
    pts.sort_by(|a, b| {
        a.x.partial_cmp(&b.x)
            .unwrap()
            .then(a.y.partial_cmp(&b.y).unwrap())
    });
    pts.dedup_by(|a, b| (a.x - b.x).abs() < 1e-12 && (a.y - b.y).abs() < 1e-12);
    pts
}

fn rotate_line_segs(lines: &[LineObs], centroid: Point<f64>, angle_deg: f64) -> Vec<LineSegRot> {
    lines
        .iter()
        .map(|l| {
            let ra = rotate_point_coord(&Coord { x: l.ax, y: l.ay }, centroid, angle_deg);
            let rb = rotate_point_coord(&Coord { x: l.bx, y: l.by }, centroid, angle_deg);
            (ra, rb)
        })
        .collect()
}

fn coarse_evaluate_angle(
    _poly: &Polygon<f64>,
    rotated: &RotatedCoords,
    obs_points_rot: &[Coord<f64>],
    obs_polygons_world: &[Polygon<f64>],
    obs_lines_rot: &[LineSegRot],
    angle: f64,
    coarse_steps: usize,
    max_ratio: f64,
    min_ratio: f64,
    centroid: (f64, f64),
) -> Option<ObsCandidate> {
    let (minx, miny, maxx, maxy) = rotated.bbox;
    if maxx <= minx || maxy <= miny || coarse_steps < 2 {
        return None;
    }

    let rad = -angle.to_radians();
    let (cos_a, sin_a) = (rad.cos(), rad.sin());

    let poly_rot_bboxes: Vec<(f64, f64, f64, f64)> = obs_polygons_world
        .iter()
        .map(|p| rotated_poly_bbox(p, centroid, cos_a, sin_a))
        .collect();

    // Build sparse base grid from obstacle coords.
    let mut xs_base: Vec<f64> = obs_points_rot.iter().map(|c| c.x).collect();
    let mut ys_base: Vec<f64> = obs_points_rot.iter().map(|c| c.y).collect();
    for &((x1, y1), (x2, y2)) in obs_lines_rot {
        xs_base.push(x1);
        xs_base.push(x2);
        ys_base.push(y1);
        ys_base.push(y2);
    }
    for &(px0, py0, px1, py1) in &poly_rot_bboxes {
        xs_base.push(px0);
        xs_base.push(px1);
        ys_base.push(py0);
        ys_base.push(py1);
    }
    xs_base.push(minx);
    xs_base.push(maxx);
    ys_base.push(miny);
    ys_base.push(maxy);
    for c in rotated.exterior.iter() {
        xs_base.push(c.x);
        ys_base.push(c.y);
    }
    xs_base.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys_base.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs_base.dedup_by(|a, b| (*a - *b).abs() < 1e-14);
    ys_base.dedup_by(|a, b| (*a - *b).abs() < 1e-14);

    // Obstacle-driven custom grid: subdivide only obstacle cells.
    const NSUB: usize = 3;
    let build = |base: &[f64], span: f64, check: &dyn Fn(f64, f64) -> bool| -> Vec<f64> {
        let mut out = Vec::with_capacity(base.len());
        for w in base.windows(2) {
            out.push(w[0]);
            let d = w[1] - w[0];
            if check(w[0], w[1]) && d > span * 1e-4 {
                for k in 1..=NSUB {
                    out.push(w[0] + d * k as f64 / (NSUB + 1) as f64);
                }
            }
        }
        out.push(*base.last().unwrap());
        while out.len() > 100 {
            let mut culled = Vec::with_capacity((out.len() + 1) / 2);
            for i in (0..out.len()).step_by(2) {
                culled.push(out[i]);
            }
            if culled.last() != out.last() {
                culled.push(*out.last().unwrap());
            }
            out = culled;
        }
        // Minimum cells for LRIH — use coarse_steps as fallback
        let min = coarse_steps.max(10);
        if out.len() < min && span > 1e-12 {
            let lo = out[0];
            let hi = out[out.len() - 1];
            out.clear();
            for i in 0..min {
                out.push(lo + (hi - lo) * i as f64 / (min - 1) as f64);
            }
        }
        out
    };
    let xs = build(&xs_base, maxx - minx, &|a, b| {
        any_rot_obstacle_in_x(a, b, obs_points_rot, obs_lines_rot, &poly_rot_bboxes)
    });
    let ys = build(&ys_base, maxy - miny, &|a, b| {
        any_rot_obstacle_in_y(a, b, obs_points_rot, obs_lines_rot, &poly_rot_bboxes)
    });

    let poly_mask = build_mask_parallel(&rotated.exterior, &rotated.holes, &xs, &ys);
    let n_cols = xs.len().saturating_sub(1);
    let n_rows = ys.len().saturating_sub(1);
    if n_cols == 0 || n_rows == 0 {
        return None;
    }

    let free_mask = build_free_mask(
        obs_points_rot,
        obs_polygons_world,
        obs_lines_rot,
        &xs,
        &ys,
        None,
        centroid,
        cos_a,
        sin_a,
    );

    let mut heights = vec![0usize; n_cols];
    let mut best_local: Option<(f64, f64, f64, f64, f64)> = None;

    for r in 0..n_rows {
        let base = r * n_cols;
        for c in 0..n_cols {
            if poly_mask[base + c] && free_mask[base + c] {
                heights[c] += 1;
            } else {
                heights[c] = 0;
            }
        }
        let (x0, y0, x1, y1, area) = lrih(&heights, &xs, &ys, r, max_ratio, min_ratio);
        if area > 0.0 {
            best_local = match best_local {
                Some((_, _, _, _, a)) if area > a => Some((x0, y0, x1, y1, area)),
                None => Some((x0, y0, x1, y1, area)),
                _ => best_local,
            };
        }
    }

    best_local.map(|(x0, y0, x1, y1, area)| ObsCandidate {
        angle,
        area,
        rect_rot: (x0, y0, x1, y1),
    })
}

fn any_rot_obstacle_in_x(
    x0: f64,
    x1: f64,
    pts: &[Coord<f64>],
    lines: &[LineSegRot],
    poly_bboxes: &[(f64, f64, f64, f64)],
) -> bool {
    for c in pts {
        if c.x >= x0 && c.x <= x1 {
            return true;
        }
    }
    for &((x1a, _), (x1b, _)) in lines {
        let lx0 = x1a.min(x1b);
        let lx1 = x1a.max(x1b);
        if lx0 <= x1 && lx1 >= x0 {
            return true;
        }
    }
    for &(px0, _, px1, _) in poly_bboxes {
        if px0 <= x1 && px1 >= x0 {
            return true;
        }
    }
    false
}

fn any_rot_obstacle_in_y(
    y0: f64,
    y1: f64,
    pts: &[Coord<f64>],
    lines: &[LineSegRot],
    poly_bboxes: &[(f64, f64, f64, f64)],
) -> bool {
    for c in pts {
        if c.y >= y0 && c.y <= y1 {
            return true;
        }
    }
    for &((_, y1a), (_, y1b)) in lines {
        let ly0 = y1a.min(y1b);
        let ly1 = y1a.max(y1b);
        if ly0 <= y1 && ly1 >= y0 {
            return true;
        }
    }
    for &(_, py0, _, py1) in poly_bboxes {
        if py0 <= y1 && py1 >= y0 {
            return true;
        }
    }
    false
}

fn rot_shrink_rect(
    mut l: f64,
    mut b: f64,
    mut r: f64,
    mut t: f64,
    pts: &[Coord<f64>],
    lines: &[LineSegRot],
    poly_bboxes: &[(f64, f64, f64, f64)],
) -> (f64, f64, f64, f64) {
    let eps = EPS;
    // Shrink from points
    loop {
        let mut best_d = f64::MAX;
        let mut edge = 0u8;
        let mut val = 0.0;
        for p in pts {
            if p.x > l + eps && p.x < r - eps && p.y > b + eps && p.y < t - eps {
                let d = (p.x - l).min(r - p.x).min(p.y - b).min(t - p.y);
                if d > 0.0 && d < best_d {
                    best_d = d;
                    if (p.x - l - d).abs() < eps {
                        edge = 1;
                        val = p.x + eps;
                    } else if (r - p.x - d).abs() < eps {
                        edge = 0;
                        val = p.x - eps;
                    } else if (p.y - b - d).abs() < eps {
                        edge = 3;
                        val = p.y + eps;
                    } else {
                        edge = 2;
                        val = p.y - eps;
                    }
                }
            }
        }
        if best_d == f64::MAX {
            break;
        }
        match edge {
            0 => l = l.max(val),
            1 => r = r.min(val),
            2 => b = b.max(val),
            3 => t = t.min(val),
            _ => {}
        }
    }
    // Shrink from polygon bboxes
    loop {
        let mut best_d = f64::MAX;
        let mut edge = 0u8;
        let mut val = 0.0;
        for &(px0, py0, px1, py1) in poly_bboxes {
            if px0 < r - eps && px1 > l + eps && py0 < t - eps && py1 > b + eps {
                let d = (r - px0 + eps)
                    .min(px1 - l + eps)
                    .min(t - py0 + eps)
                    .min(py1 - b + eps);
                if d > 0.0 && d < best_d {
                    best_d = d;
                    if ((r - px0 + eps) - d).abs() < eps {
                        edge = 1;
                        val = px0 - eps;
                    } else if ((px1 - l + eps) - d).abs() < eps {
                        edge = 0;
                        val = px1 + eps;
                    } else if ((t - py0 + eps) - d).abs() < eps {
                        edge = 3;
                        val = py0 - eps;
                    } else {
                        edge = 2;
                        val = py1 + eps;
                    }
                }
            }
        }
        if best_d == f64::MAX {
            break;
        }
        match edge {
            0 => l = l.max(val),
            1 => r = r.min(val),
            2 => b = b.max(val),
            3 => t = t.min(val),
            _ => {}
        }
    }
    (l.max(0.0).min(r), b.max(0.0).min(t), r, t)
}

fn fine_solve_angle(
    _poly: &Polygon<f64>,
    rotated: &RotatedCoords,
    obs_points_rot: &[Coord<f64>],
    obs_polygons_world: &[Polygon<f64>],
    obs_lines_rot: &[LineSegRot],
    coarse: &ObsCandidate,
    max_ratio: f64,
    min_ratio: f64,
    centroid: (f64, f64),
    options: &LirObstaclesOptions,
) -> Option<ObsCandidate> {
    let (minx, miny, maxx, maxy) = rotated.bbox;
    let span_x = maxx - minx;
    let span_y = maxy - miny;
    if span_x <= 0.0 || span_y <= 0.0 {
        return Some(*coarse);
    }

    let rad = -coarse.angle.to_radians();
    let (cos_a, sin_a) = (rad.cos(), rad.sin());

    let mut xs_raw: Vec<f64> = obs_points_rot.iter().map(|c| c.x).collect();
    let mut ys_raw: Vec<f64> = obs_points_rot.iter().map(|c| c.y).collect();
    for &((x1, y1), (x2, y2)) in obs_lines_rot {
        xs_raw.push(x1);
        xs_raw.push(x2);
        ys_raw.push(y1);
        ys_raw.push(y2);
    }
    for c in rotated.exterior.iter() {
        xs_raw.push(c.x);
        ys_raw.push(c.y);
    }
    xs_raw.push(minx);
    xs_raw.push(maxx);
    ys_raw.push(miny);
    ys_raw.push(maxy);

    // Add polygon obstacle bbox edges so the grid has cells directly at obstacle
    // boundaries — without this, polygon obstacles are resolved only via cell-centre
    // containment checks on a grid that ignores their edges.
    for p in obs_polygons_world {
        let (p_minx, p_maxx, p_miny, p_maxy) = rotated_poly_bbox(p, centroid, cos_a, sin_a);
        xs_raw.push(p_minx);
        xs_raw.push(p_maxx);
        ys_raw.push(p_miny);
        ys_raw.push(p_maxy);
    }

    xs_raw.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys_raw.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs_raw.dedup_by(|a, b| (*a - *b).abs() < 1e-14);
    ys_raw.dedup_by(|a, b| (*a - *b).abs() < 1e-14);

    // Collect rotated bboxes of polygon obstacles for overlap checks.
    let poly_rot_bboxes: Vec<(f64, f64, f64, f64)> = obs_polygons_world
        .iter()
        .map(|p| rotated_poly_bbox(p, centroid, cos_a, sin_a))
        .collect();

    // Obstacle-driven custom grid: subdivide only cells that overlap an obstacle.
    // Open-space cells stay at obstacle-coordinate size.
    const N_SUB: usize = 6;
    let xs_sparse = xs_raw.clone();
    let ys_sparse = ys_raw.clone();
    xs_raw.clear();
    ys_raw.clear();
    for w in xs_sparse.windows(2) {
        xs_raw.push(w[0]);
        let sub =
            any_rot_obstacle_in_x(w[0], w[1], obs_points_rot, obs_lines_rot, &poly_rot_bboxes);
        if sub && w[1] - w[0] > span_x * 0.001 {
            for k in 1..=N_SUB {
                xs_raw.push(w[0] + (w[1] - w[0]) * k as f64 / (N_SUB + 1) as f64);
            }
        }
    }
    xs_raw.push(*xs_sparse.last().unwrap());
    for w in ys_sparse.windows(2) {
        ys_raw.push(w[0]);
        let sub =
            any_rot_obstacle_in_y(w[0], w[1], obs_points_rot, obs_lines_rot, &poly_rot_bboxes);
        if sub && w[1] - w[0] > span_y * 0.001 {
            for k in 1..=N_SUB {
                ys_raw.push(w[0] + (w[1] - w[0]) * k as f64 / (N_SUB + 1) as f64);
            }
        }
    }
    ys_raw.push(*ys_sparse.last().unwrap());

    // Minimum cells fallback for no-obstacle case.
    if xs_raw.len() < 12 {
        xs_raw.clear();
        for i in 0..12 {
            xs_raw.push(minx + span_x * i as f64 / 11.0);
        }
    }
    if ys_raw.len() < 12 {
        ys_raw.clear();
        for i in 0..12 {
            ys_raw.push(miny + span_y * i as f64 / 11.0);
        }
    }

    let n_cols = xs_raw.len().saturating_sub(1);
    let n_rows = ys_raw.len().saturating_sub(1);
    if n_cols < 1 || n_rows < 1 {
        return None;
    }

    let poly_mask = build_mask_parallel(&rotated.exterior, &rotated.holes, &xs_raw, &ys_raw);
    let free_mask = build_free_mask(
        obs_points_rot,
        obs_polygons_world,
        obs_lines_rot,
        &xs_raw,
        &ys_raw,
        None,
        centroid,
        cos_a,
        sin_a,
    );

    let mut heights = vec![0usize; n_cols];
    let mut best_local: Option<(f64, f64, f64, f64, f64)> = None;

    let (sx0, sy0, sx1, sy1) = coarse.rect_rot;
    if sx1 > sx0 && sy1 > sy0 {
        best_local = Some((sx0, sy0, sx1, sy1, (sx1 - sx0) * (sy1 - sy0)));
    }

    for r in 0..n_rows {
        let base = r * n_cols;
        for c in 0..n_cols {
            if poly_mask[base + c] && free_mask[base + c] {
                heights[c] += 1;
            } else {
                heights[c] = 0;
            }
        }
        let (x0, y0, x1, y1, area) = lrih_vp(&heights, &xs_raw, &ys_raw, r, max_ratio, min_ratio);
        if area > 0.0 {
            best_local = match best_local {
                Some((_, _, _, _, a)) if area > a => Some((x0, y0, x1, y1, area)),
                None => Some((x0, y0, x1, y1, area)),
                _ => best_local,
            };
        }
    }

    let (rx0, ry0, rx1, ry1, _area) = best_local?;

    let rot_poly = Polygon::new(
        LineString::from(rotated.exterior.clone()),
        rotated
            .holes
            .iter()
            .map(|h| LineString::from(h.clone()))
            .collect(),
    );

    let (ex0, ey0, ex1, ey1) =
        expand_rect_to_boundary(&rot_poly, rx0, ry0, rx1, ry1, max_ratio, min_ratio);

    let rect_poly = Polygon::new(
        LineString::from(vec![
            Coord { x: ex0, y: ey0 },
            Coord { x: ex1, y: ey0 },
            Coord { x: ex1, y: ey1 },
            Coord { x: ex0, y: ey1 },
            Coord { x: ex0, y: ey0 },
        ]),
        vec![],
    );

    if let Some((cert_poly, cert_area)) = certify_and_adjust(
        &rot_poly,
        &rect_poly,
        max_ratio,
        options.cert_eps,
        options.cert_max_shrink,
    ) {
        let cert_bb = cert_poly.bounding_rect()?;
        let cert_x0 = cert_bb.min().x;
        let cert_y0 = cert_bb.min().y;
        let cert_x1 = cert_bb.max().x;
        let cert_y1 = cert_bb.max().y;
        let exp_area = (ex1 - ex0) * (ey1 - ey0);

        let _final_area = if cert_area > exp_area {
            cert_area
        } else {
            exp_area
        };
        let (fx0, fy0, fx1, fy1) = if cert_area >= exp_area {
            (cert_x0, cert_y0, cert_x1, cert_y1)
        } else {
            (ex0, ey0, ex1, ey1)
        };

        // Shrink from obstacles in the rotated frame so the returned
        // rectangle avoids them even if the grid mask missed some.
        let (sx0, sy0, sx1, sy1) = rot_shrink_rect(
            fx0,
            fy0,
            fx1,
            fy1,
            obs_points_rot,
            obs_lines_rot,
            &poly_rot_bboxes,
        );
        let shrunk_area = (sx1 - sx0) * (sy1 - sy0).max(0.0);

        return Some(ObsCandidate {
            angle: coarse.angle,
            area: shrunk_area,
            rect_rot: (sx0, sy0, sx1, sy1),
        });
    }

    let (sx0, sy0, sx1, sy1) = rot_shrink_rect(
        rx0,
        ry0,
        rx1,
        ry1,
        obs_points_rot,
        obs_lines_rot,
        &poly_rot_bboxes,
    );
    Some(ObsCandidate {
        angle: coarse.angle,
        area: (sx1 - sx0) * (sy1 - sy0).max(0.0),
        rect_rot: (sx0, sy0, sx1, sy1),
    })
}

pub fn solve_lir_obstacles_oriented(
    poly: &Polygon<f64>,
    obstacles: &[ObstacleInput],
    options: &LirObstaclesOptions,
) -> Result<LirObstaclesResult> {
    if poly.exterior().0.len() < 3 || poly.bounding_rect().is_none() || poly.unsigned_area() < 1e-12
    {
        return Err(LirError::InvalidPolygon(
            "Polygon has <3 vertices or zero area".into(),
        ));
    }

    let centroid_pt: Point<f64> = poly
        .centroid()
        .map(|c| c.into())
        .unwrap_or(Point::new(0.0, 0.0));
    let cent = (centroid_pt.x(), centroid_pt.y());

    let mut g = group_obstacles(obstacles);

    if options.line_thickness > 0.0 {
        let buffered = buffer_lines_to_polygons(&g.lines, options.line_thickness);
        let mut all_poly_obs: Vec<ObstacleInput> = g
            .polygons
            .iter()
            .map(|p| ObstacleInput::Polygon(p.poly.clone()))
            .collect();
        for bp in &buffered {
            all_poly_obs.push(ObstacleInput::Polygon(bp.poly.clone()));
        }
        for pt in &g.points {
            all_poly_obs.push(ObstacleInput::Point(Coord { x: pt.x, y: pt.y }));
        }
        g = group_obstacles(&all_poly_obs);
    }

    let mut obs_points: Vec<Coord<f64>> = extract_obstacle_points(
        &g.polygons
            .iter()
            .map(|p| p.poly.clone())
            .collect::<Vec<_>>(),
    );
    // Point obstacles are handled post-solve by shrink_away_from_points,
    // not via grid cells (zero-area points are grid-invisible).
    for pt in &g.points {
        obs_points.push(Coord { x: pt.x, y: pt.y });
    }

    let coarse_steps = options.grid_coarse.max(8);
    let top_k = options.top_k.max(1);

    let all_angles = generate_angles(poly, 24);

    let mut candidates: Vec<ObsCandidate> = all_angles
        .par_iter()
        .filter_map(|&angle| {
            let rotated = rotate_coords_only(poly, angle);
            let rot_pts = rotate_points(&obs_points, centroid_pt, angle);
            let rot_lines = rotate_line_segs(&g.lines, centroid_pt, angle);
            coarse_evaluate_angle(
                poly,
                &rotated,
                &rot_pts,
                &g.polygons
                    .iter()
                    .map(|p| p.poly.clone())
                    .collect::<Vec<_>>(),
                &rot_lines,
                angle,
                coarse_steps,
                options.max_ratio,
                options.min_ratio,
                cent,
            )
        })
        .collect();

    if candidates.is_empty() {
        if options.always_return {
            return Ok(LirObstaclesResult {
                best_effort: true,
                ..LirObstaclesResult::empty()
            });
        }
        return Err(LirError::NoRectangleFound);
    }

    candidates.sort_by(|a, b| {
        b.area
            .partial_cmp(&a.area)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let best_angles: Vec<f64> = candidates.iter().map(|c| c.angle).take(3).collect();
    let refinement: Vec<f64> = best_angles
        .iter()
        .flat_map(|&base| vec![base - 1.0, base + 1.0])
        .filter(|&a| a >= 0.0 && a <= 90.0)
        .filter(|a| !all_angles.iter().any(|ta| (ta - a).abs() < 0.5))
        .collect();

    for &angle in &refinement {
        let rotated = rotate_coords_only(poly, angle);
        let rot_pts = rotate_points(&obs_points, centroid_pt, angle);
        let rot_lines = rotate_line_segs(&g.lines, centroid_pt, angle);
        if let Some(c) = coarse_evaluate_angle(
            poly,
            &rotated,
            &rot_pts,
            &g.polygons
                .iter()
                .map(|p| p.poly.clone())
                .collect::<Vec<_>>(),
            &rot_lines,
            angle,
            coarse_steps,
            options.max_ratio,
            options.min_ratio,
            cent,
        ) {
            candidates.push(c);
        }
    }

    candidates.sort_by(|a, b| {
        b.area
            .partial_cmp(&a.area)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut seen: Vec<f64> = Vec::new();
    candidates.retain(|c| {
        if seen.iter().any(|&s| (c.angle - s).abs() < 2.0) {
            false
        } else {
            seen.push(c.angle);
            true
        }
    });

    let top_k = candidates.len().min(top_k);
    if top_k == 0 {
        if options.always_return {
            return Ok(LirObstaclesResult {
                best_effort: true,
                ..LirObstaclesResult::empty()
            });
        }
        return Err(LirError::NoRectangleFound);
    }

    let fine_results: Vec<Option<ObsCandidate>> = candidates[..top_k]
        .par_iter()
        .map(|cand| {
            let rotated = rotate_coords_only(poly, cand.angle);
            let rot_pts = rotate_points(&obs_points, centroid_pt, cand.angle);
            let rot_lines = rotate_line_segs(&g.lines, centroid_pt, cand.angle);
            fine_solve_angle(
                poly,
                &rotated,
                &rot_pts,
                &g.polygons
                    .iter()
                    .map(|p| p.poly.clone())
                    .collect::<Vec<_>>(),
                &rot_lines,
                cand,
                options.max_ratio,
                options.min_ratio,
                cent,
                options,
            )
        })
        .collect();

    let best = fine_results
        .into_iter()
        .flatten()
        .max_by(|a, b| {
            a.area
                .partial_cmp(&b.area)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .ok_or_else(|| {
            if options.always_return {
                LirError::NoRectangleFound
            } else {
                LirError::NoRectangleFound
            }
        })?;

    if best.area <= EPS {
        return if options.always_return {
            Ok(LirObstaclesResult {
                best_effort: true,
                ..LirObstaclesResult::empty()
            })
        } else {
            Err(LirError::NoRectangleFound)
        };
    }

    let raw_poly = Polygon::new(
        LineString::from(vec![
            rotate_point(best.rect_rot.0, best.rect_rot.1, best.angle, &centroid_pt),
            rotate_point(best.rect_rot.2, best.rect_rot.1, best.angle, &centroid_pt),
            rotate_point(best.rect_rot.2, best.rect_rot.3, best.angle, &centroid_pt),
            rotate_point(best.rect_rot.0, best.rect_rot.3, best.angle, &centroid_pt),
            rotate_point(best.rect_rot.0, best.rect_rot.1, best.angle, &centroid_pt),
        ]),
        vec![],
    );

    let bb = raw_poly.bounding_rect().unwrap();
    let area = best.area;
    let best_effort = area <= EPS;

    Ok(LirObstaclesResult {
        rect: Some(Rectangle {
            x_min: bb.min().x,
            y_min: bb.min().y,
            x_max: bb.max().x,
            y_max: bb.max().y,
        }),
        rect_polygon: Some(raw_poly),
        area,
        angle_deg: best.angle,
        best_effort,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    fn rp(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<f64> {
        Polygon::new(
            LineString::from(vec![
                coord! { x: x0, y: y0 },
                coord! { x: x1, y: y0 },
                coord! { x: x1, y: y1 },
                coord! { x: x0, y: y1 },
                coord! { x: x0, y: y0 },
            ]),
            vec![],
        )
    }

    fn opts() -> LirObstaclesOptions {
        LirObstaclesOptions::default()
    }

    #[test]
    fn no_obstacles_square() {
        let poly = rp(0.0, 0.0, 10.0, 10.0);
        let result = solve_lir_obstacles_oriented(&poly, &[], &opts()).unwrap();
        assert!(result.area > 90.0, "area too small: {}", result.area);
    }

    #[test]
    fn central_polygon_obstacle() {
        let poly = rp(0.0, 0.0, 10.0, 10.0);
        let obs = vec![ObstacleInput::Polygon(rp(4.0, 4.0, 6.0, 6.0))];
        let result = solve_lir_obstacles_oriented(&poly, &obs, &opts()).unwrap();
        assert!(result.area > 30.0);
        assert!(result.area < 100.0);
    }

    #[test]
    fn point_obstacle() {
        let poly = rp(0.0, 0.0, 10.0, 10.0);
        let obs = vec![ObstacleInput::Point(coord! { x: 5.0, y: 5.0 })];
        let result = solve_lir_obstacles_oriented(&poly, &obs, &opts()).unwrap();
        assert!(result.area > 20.0);
        assert!(result.area < 100.0);
    }

    #[test]
    fn line_obstacle() {
        let poly = rp(0.0, 0.0, 10.0, 10.0);
        let line = LineString::from(vec![coord! { x: 5.0, y: 0.0 }, coord! { x: 5.0, y: 10.0 }]);
        let obs = vec![ObstacleInput::Line(line)];
        let result = solve_lir_obstacles_oriented(&poly, &obs, &opts()).unwrap();
        assert!(result.area > 30.0);
    }

    #[test]
    fn triangular_polygon() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! { x: 0.0, y: 0.0 },
                coord! { x: 10.0, y: 0.0 },
                coord! { x: 0.0, y: 10.0 },
                coord! { x: 0.0, y: 0.0 },
            ]),
            vec![],
        );
        let obs = vec![ObstacleInput::Point(coord! { x: 3.0, y: 3.0 })];
        let result = solve_lir_obstacles_oriented(&poly, &obs, &opts()).unwrap();
        assert!(result.area > 0.0);
    }

    #[test]
    fn degenerate_polygon() {
        let flat = Polygon::new(
            LineString::from(vec![
                coord! { x: 0.0, y: 0.0 },
                coord! { x: 5.0, y: 0.0 },
                coord! { x: 0.0, y: 0.0 },
            ]),
            vec![],
        );
        let result = solve_lir_obstacles_oriented(&flat, &[], &opts());
        assert!(result.is_err());
    }
}
