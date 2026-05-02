use crate::mic::input::SegmentIndex;

/// Nearest-boundary distance queries over segment table.
#[derive(Debug, Clone)]
pub struct NearestBoundaryIndex {
    segments: SegmentIndex,
}

impl NearestBoundaryIndex {
    pub fn new(segments: SegmentIndex) -> Self {
        Self { segments }
    }

    pub fn nearest_distance_sq(&self, x: f64, y: f64) -> Option<(f64, usize)> {
        if self.segments.is_empty() {
            return None;
        }

        let mut best_sq = f64::INFINITY;
        let mut best_idx = 0usize;

        for seg_idx in 0..self.segments.len() {
            let bbox_lb = point_to_bbox_distance_sq(
                x,
                y,
                self.segments.bbox_minx[seg_idx],
                self.segments.bbox_miny[seg_idx],
                self.segments.bbox_maxx[seg_idx],
                self.segments.bbox_maxy[seg_idx],
            );
            if bbox_lb > best_sq {
                continue;
            }

            let d_sq = self.segments.point_segment_distance_sq(seg_idx, x, y);
            if d_sq < best_sq {
                best_sq = d_sq;
                best_idx = seg_idx;
            }
        }

        Some((best_sq, best_idx))
    }

    pub fn supporting_segments(
        &self,
        x: f64,
        y: f64,
        min_dist_sq: f64,
        eps: f64,
    ) -> Vec<usize> {
        let mut supports = Vec::new();
        let max_dist_sq = min_dist_sq + eps.abs().max(1e-14);

        for seg_idx in 0..self.segments.len() {
            let bbox_lb = point_to_bbox_distance_sq(
                x,
                y,
                self.segments.bbox_minx[seg_idx],
                self.segments.bbox_miny[seg_idx],
                self.segments.bbox_maxx[seg_idx],
                self.segments.bbox_maxy[seg_idx],
            );
            if bbox_lb > max_dist_sq {
                continue;
            }
            let d_sq = self.segments.point_segment_distance_sq(seg_idx, x, y);
            if d_sq <= max_dist_sq {
                supports.push(seg_idx);
            }
        }

        supports
    }
}

fn point_to_bbox_distance_sq(x: f64, y: f64, min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> f64 {
    let dx = if x < min_x {
        min_x - x
    } else if x > max_x {
        x - max_x
    } else {
        0.0
    };
    let dy = if y < min_y {
        min_y - y
    } else if y > max_y {
        y - max_y
    } else {
        0.0
    };
    dx * dx + dy * dy
}
