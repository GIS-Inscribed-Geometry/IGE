use geo_types::Polygon;

/// Winding number index for point-in-polygon tests.
pub struct WindingIndex;

impl WindingIndex {
    pub fn new() -> Self {
        WindingIndex
    }

    pub fn from_polygon(_poly: &Polygon<f64>) -> Self {
        WindingIndex
    }
}
