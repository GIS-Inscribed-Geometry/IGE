use geo::Contains;
use geo_types::{Coord, Point, Polygon};

/// Build a mask where a cell is TRUE (free) if it contains NO obstacle points,
/// its center is NOT inside any obstacle polygon, AND its center lies inside the
/// feasible quadrilateral (rotated world bbox).
///
/// Obstacle polygons are provided in **world space** — cell centers are inverse-
/// rotated back to world space for the containment test, avoiding the need to
/// re-rotate polygons for each evaluation angle.
///
/// For each obstacle polygon, we find its AABB in the rotated frame (by rotating
/// its vertices), then only check cells within that bbox range. This avoids the
/// O(cells × polygons) cost of checking every cell against every polygon.
///
/// No internal parallelism — the caller is expected to parallelize at the angle
/// level (outer loop), avoiding nested rayon oversubscription.
/// Line segments in the rotated frame, each as ((x1,y1),(x2,y2)).
pub type LineSegRot = ((f64, f64), (f64, f64));

pub fn build_free_mask(
    obs_points: &[Coord<f64>],
    obs_polygons_world: &[Polygon<f64>],
    obs_lines_rot: &[LineSegRot],
    xs: &[f64],
    ys: &[f64],
    quad: Option<&[Coord<f64>; 4]>,
    centroid: (f64, f64),
    cos_a: f64,
    sin_a: f64,
) -> Vec<bool> {
    let n_cols = xs.len().saturating_sub(1);
    let n_rows = ys.len().saturating_sub(1);
    if n_cols == 0 || n_rows == 0 {
        return Vec::new();
    }

    let total = n_cols * n_rows;
    let mut mask = vec![true; total];

    // --- Mark cells containing obstacle points as blocked ---
    for pt in obs_points {
        let px = pt.x;
        let py = pt.y;

        let mut col_start = None;
        let mut col_end = None;
        for c in 0..n_cols {
            if px >= xs[c] && px < xs[c + 1] {
                col_start = Some(c);
                col_end = Some(c + 1);
                break;
            }
        }
        if col_start.is_none() {
            for c in 0..=n_cols {
                if (px - xs[c]).abs() < 1e-12 {
                    col_start = Some(c.saturating_sub(1));
                    col_end = Some((c + 1).min(n_cols));
                    break;
                }
            }
        }

        let mut row_start = None;
        let mut row_end = None;
        for r in 0..n_rows {
            if py >= ys[r] && py < ys[r + 1] {
                row_start = Some(r);
                row_end = Some(r + 1);
                break;
            }
        }
        if row_start.is_none() {
            for r in 0..=n_rows {
                if (py - ys[r]).abs() < 1e-12 {
                    row_start = Some(r.saturating_sub(1));
                    row_end = Some((r + 1).min(n_rows));
                    break;
                }
            }
        }

        if let (Some(cs), Some(ce), Some(rs), Some(re)) = (col_start, col_end, row_start, row_end) {
            for r in rs..re {
                for c in cs..ce {
                    mask[r * n_cols + c] = false;
                }
            }
        }
    }

    // --- Mark cells whose inverse-rotated center lies inside any obstacle polygon ---
    // Per-polygon strategy: compute each polygon's bbox in the rotated frame,
    // then only check grid cells overlapping that bbox.
    for obs_poly in obs_polygons_world {
        let (min_x, max_x, min_y, max_y) = rotated_poly_bbox(obs_poly, centroid, cos_a, sin_a);

        let mut c_start = n_cols;
        let mut c_end = 0;
        for c in 0..n_cols {
            if xs[c + 1] > min_x && xs[c] < max_x {
                if c < c_start {
                    c_start = c;
                }
                c_end = c + 1;
            }
        }
        if c_start >= c_end {
            continue;
        }

        let mut r_start = n_rows;
        let mut r_end = 0;
        for r in 0..n_rows {
            if ys[r + 1] > min_y && ys[r] < max_y {
                if r < r_start {
                    r_start = r;
                }
                r_end = r + 1;
            }
        }
        if r_start >= r_end {
            continue;
        }

        for r in r_start..r_end {
            let cy = (ys[r] + ys[r + 1]) * 0.5;
            for c in c_start..c_end {
                let idx = r * n_cols + c;
                if !mask[idx] {
                    continue;
                }
                let cx = (xs[c] + xs[c + 1]) * 0.5;
                let dx = cx - centroid.0;
                let dy = cy - centroid.1;
                let wx = centroid.0 + dx * cos_a + dy * sin_a;
                let wy = centroid.1 - dx * sin_a + dy * cos_a;
                if obs_poly.contains(&Point::new(wx, wy)) {
                    mask[idx] = false;
                }
            }
        }
    }

    // --- Mark cells intersected by obstacle line segments ---
    for &((lx1, ly1), (lx2, ly2)) in obs_lines_rot {
        let lminx = lx1.min(lx2);
        let lmaxx = lx1.max(lx2);
        let lminy = ly1.min(ly2);
        let lmaxy = ly1.max(ly2);
        let mut c_start = n_cols;
        let mut c_end = 0;
        for c in 0..n_cols {
            if xs[c + 1] >= lminx && xs[c] <= lmaxx {
                if c < c_start {
                    c_start = c;
                }
                c_end = c + 1;
            }
        }
        if c_start >= c_end {
            continue;
        }
        let mut r_start = n_rows;
        let mut r_end = 0;
        for r in 0..n_rows {
            if ys[r + 1] >= lminy && ys[r] <= lmaxy {
                if r < r_start {
                    r_start = r;
                }
                r_end = r + 1;
            }
        }
        if r_start >= r_end {
            continue;
        }
        for r in r_start..r_end {
            for c in c_start..c_end {
                let idx = r * n_cols + c;
                if !mask[idx] {
                    continue;
                }
                if line_intersects_aabb(lx1, ly1, lx2, ly2, xs[c], xs[c + 1], ys[r], ys[r + 1]) {
                    mask[idx] = false;
                }
            }
        }
    }

    // --- Mark cells whose center is outside the feasible quadrilateral ---
    if let Some(qc) = quad {
        for r in 0..n_rows {
            let cy = (ys[r] + ys[r + 1]) * 0.5;
            for c in 0..n_cols {
                let cx = (xs[c] + xs[c + 1]) * 0.5;
                if !point_in_convex_quad(cx, cy, qc) {
                    mask[r * n_cols + c] = false;
                }
            }
        }
    }

    mask
}

/// Compute the AABB of all polygon vertices (exterior + holes) after rotating
/// them into the grid frame.
pub(crate) fn rotated_poly_bbox(
    poly: &Polygon<f64>,
    centroid: (f64, f64),
    cos_a: f64,
    sin_a: f64,
) -> (f64, f64, f64, f64) {
    let mut min_x = f64::MAX;
    let mut max_x = f64::MIN;
    let mut min_y = f64::MAX;
    let mut max_y = f64::MIN;

    for c in poly.exterior().coords() {
        let dx = c.x - centroid.0;
        let dy = c.y - centroid.1;
        let rx = centroid.0 + dx * cos_a - dy * sin_a;
        let ry = centroid.1 + dx * sin_a + dy * cos_a;
        if rx < min_x {
            min_x = rx;
        }
        if rx > max_x {
            max_x = rx;
        }
        if ry < min_y {
            min_y = ry;
        }
        if ry > max_y {
            max_y = ry;
        }
    }
    for hole in poly.interiors() {
        for c in hole.coords() {
            let dx = c.x - centroid.0;
            let dy = c.y - centroid.1;
            let rx = centroid.0 + dx * cos_a - dy * sin_a;
            let ry = centroid.1 + dx * sin_a + dy * cos_a;
            if rx < min_x {
                min_x = rx;
            }
            if rx > max_x {
                max_x = rx;
            }
            if ry < min_y {
                min_y = ry;
            }
            if ry > max_y {
                max_y = ry;
            }
        }
    }
    (min_x, max_x, min_y, max_y)
}

/// Check if point (px, py) lies inside a convex quadrilateral
/// (assumed CCW winding, the rotated world bounding box).
fn point_in_convex_quad(px: f64, py: f64, quad: &[Coord<f64>; 4]) -> bool {
    for i in 0..4 {
        let a = quad[i];
        let b = quad[(i + 1) % 4];
        let cross = (b.x - a.x) * (py - a.y) - (b.y - a.y) * (px - a.x);
        if cross < -1e-12 {
            return false;
        }
    }
    true
}

/// Liang-Barsky line segment vs. axis-aligned bounding box intersection test.
fn line_intersects_aabb(
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    xmin: f64,
    xmax: f64,
    ymin: f64,
    ymax: f64,
) -> bool {
    let dx = x2 - x1;
    let dy = y2 - y1;
    let mut t_min: f64 = 0.0;
    let mut t_max: f64 = 1.0;
    let edges = [
        (-dx, x1 - xmin),
        (dx, xmax - x1),
        (-dy, y1 - ymin),
        (dy, ymax - y1),
    ];
    for &(p, q) in &edges {
        if p.abs() < 1e-15 {
            if q < 0.0 {
                return false;
            }
        } else {
            let t = q / p;
            if p < 0.0 {
                t_min = t_min.max(t);
            } else {
                t_max = t_max.min(t);
            }
        }
    }
    t_min <= t_max
}
