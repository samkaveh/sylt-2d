use crate::math_utils::Vec2;

pub struct Metaball {
    pub position: Vec2,
    pub radius: f32,
    pub strength: f32,
}

impl Metaball {
    pub fn new(position: Vec2, radius: f32, strength: f32) -> Self {
        Self {
            position,
            radius,
            strength,
        }
    }

    pub fn field_value(&self, point: Vec2) -> f32 {
        let diff = point - self.position;
        let dist_sq = diff.dot(diff);
        if dist_sq < f32::EPSILON {
            return f32::MAX;
        }
        self.strength * (self.radius * self.radius) / dist_sq
    }
}

pub fn sample_field(metaballs: &[Metaball], point: Vec2) -> f32 {
    metaballs.iter().map(|m| m.field_value(point)).sum()
}

pub fn marching_squares(
    metaballs: &[Metaball],
    bounds_min: Vec2,
    bounds_max: Vec2,
    resolution: usize,
    threshold: f32,
) -> Vec<Vec<(f32, f32)>> {
    let width = bounds_max.x - bounds_min.x;
    let height = bounds_max.y - bounds_min.y;
    let cell_w = width / resolution as f32;
    let cell_h = height / resolution as f32;

    let mut grid = vec![vec![0.0f32; resolution + 1]; resolution + 1];
    for j in 0..=resolution {
        for i in 0..=resolution {
            let x = bounds_min.x + i as f32 * cell_w;
            let y = bounds_min.y + j as f32 * cell_h;
            grid[j][i] = sample_field(metaballs, Vec2::new(x, y));
        }
    }

    let mut segments: Vec<(Vec2, Vec2)> = Vec::new();

    for j in 0..resolution {
        for i in 0..resolution {
            let tl = grid[j + 1][i];
            let tr = grid[j + 1][i + 1];
            let br = grid[j][i + 1];
            let bl = grid[j][i];

            let x0 = bounds_min.x + i as f32 * cell_w;
            let x1 = x0 + cell_w;
            let y0 = bounds_min.y + j as f32 * cell_h;
            let y1 = y0 + cell_h;

            let mut case_index = 0;
            if tl >= threshold { case_index |= 8; }
            if tr >= threshold { case_index |= 4; }
            if br >= threshold { case_index |= 2; }
            if bl >= threshold { case_index |= 1; }

            let interp = |v1: f32, v2: f32, p1: Vec2, p2: Vec2| -> Vec2 {
                let t = (threshold - v1) / (v2 - v1);
                p1 + (p2 - p1) * t
            };

            let top = interp(tl, tr, Vec2::new(x0, y1), Vec2::new(x1, y1));
            let right = interp(tr, br, Vec2::new(x1, y1), Vec2::new(x1, y0));
            let bottom = interp(bl, br, Vec2::new(x0, y0), Vec2::new(x1, y0));
            let left = interp(tl, bl, Vec2::new(x0, y1), Vec2::new(x0, y0));

            match case_index {
                1 => segments.push((left, bottom)),
                2 => segments.push((bottom, right)),
                3 => segments.push((left, right)),
                4 => segments.push((top, right)),
                5 => {
                    segments.push((left, top));
                    segments.push((bottom, right));
                }
                6 => segments.push((top, bottom)),
                7 => segments.push((left, top)),
                8 => segments.push((top, left)),
                9 => segments.push((top, bottom)),
                10 => {
                    segments.push((top, right));
                    segments.push((left, bottom));
                }
                11 => segments.push((top, right)),
                12 => segments.push((left, right)),
                13 => segments.push((bottom, right)),
                14 => segments.push((left, bottom)),
                _ => {}
            }
        }
    }

    chain_segments(segments)
}

fn chain_segments(segments: Vec<(Vec2, Vec2)>) -> Vec<Vec<(f32, f32)>> {
    if segments.is_empty() {
        return Vec::new();
    }

    let eps = 0.001;
    let eq = |a: Vec2, b: Vec2| -> bool { (a.x - b.x).abs() < eps && (a.y - b.y).abs() < eps };

    let mut remaining: Vec<(Vec2, Vec2)> = segments;
    let mut polygons: Vec<Vec<(f32, f32)>> = Vec::new();

    while !remaining.is_empty() {
        let mut poly: Vec<(f32, f32)> = Vec::new();
        let first = remaining.remove(0);
        poly.push((first.0.x, first.0.y));
        poly.push((first.1.x, first.1.y));
        let mut current_end = first.1;

        loop {
            let mut found = false;
            for i in 0..remaining.len() {
                let seg = remaining[i];
                if eq(current_end, seg.0) {
                    poly.push((seg.1.x, seg.1.y));
                    current_end = seg.1;
                    remaining.remove(i);
                    found = true;
                    break;
                } else if eq(current_end, seg.1) {
                    poly.push((seg.0.x, seg.0.y));
                    current_end = seg.0;
                    remaining.remove(i);
                    found = true;
                    break;
                }
            }
            if !found {
                break;
            }
        }

        if eq(current_end, Vec2::new(poly[0].0, poly[0].1)) {
            polygons.push(poly);
        } else if poly.len() > 2 {
            polygons.push(poly);
        }
    }

    polygons
}
