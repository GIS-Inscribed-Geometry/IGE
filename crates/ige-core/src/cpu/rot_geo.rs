use geo_types::Polygon;

pub fn rotate_polygon(poly: &Polygon<f64>, angle_deg: f64) -> Polygon<f64> {
    if angle_deg.abs() < 1e-9 {
        return poly.clone();
    }
    
    let angle_rad = angle_deg.to_radians();
    let cos_a = angle_rad.cos();
    let sin_a = angle_rad.sin();
    
    let ext_coords: Vec<_> = poly.exterior().0.iter()
        .map(|c| geo_types::Coord {
            x: c.x * cos_a - c.y * sin_a,
            y: c.x * sin_a + c.y * cos_a,
        })
        .collect();
    let ext = geo_types::LineString::from(ext_coords);
    
    let interiors: Vec<_> = poly.interiors().iter().map(|r| {
        let coords: Vec<_> = r.0.iter()
            .map(|c| geo_types::Coord {
                x: c.x * cos_a - c.y * sin_a,
                y: c.x * sin_a + c.y * cos_a,
            })
            .collect();
        geo_types::LineString::from(coords)
    }).collect();
    
    geo_types::Polygon::new(ext, interiors)
}