use geo_types::Coord;

/// Build a mask where a cell is TRUE (free) if it contains NO obstacle points
/// AND its center lies inside the feasible quadrilateral (rotated world bbox).
///
/// For the coarse uniform grid: a cell is blocked if any obstacle point
/// lies within its bounds, or if its center is outside the quadrilateral.
///
/// For the fine vertex grid: grid lines pass through obstacle-point
/// coordinates, so points fall exactly on boundaries.  We mark all adjacent
/// cells as blocked to be safe.
pub fn build_free_mask(
    obs_points: &[Coord<f64>],
    xs: &[f64],
    ys: &[f64],
    quad: Option<&[Coord<f64>; 4]>,
) -> Vec<bool> {
    let n_cols = xs.len().saturating_sub(1);
    let n_rows = ys.len().saturating_sub(1);
    if n_cols == 0 || n_rows == 0 {
        return Vec::new();
    }

    let mut mask = vec![true; n_cols * n_rows];

    // Mark cells containing obstacle points as blocked
    if !obs_points.is_empty() {
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

            if let (Some(cs), Some(ce), Some(rs), Some(re)) =
                (col_start, col_end, row_start, row_end)
            {
                for r in rs..re {
                    for c in cs..ce {
                        mask[r * n_cols + c] = false;
                    }
                }
            }
        }
    }

    // Mark cells whose center is outside the feasible quadrilateral as blocked
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
