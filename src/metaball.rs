use crate::body::Body;
use crate::math_utils::{Aabb, Vec2};

#[derive(Clone, Copy)]
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

pub fn sample_field_with_obstacles(metaballs: &[Metaball], point: Vec2, obstacles: &[Body]) -> f32 {
    for obstacle in obstacles {
        if obstacle.point_inside(point) {
            return 0.0;
        }
    }
    metaballs.iter().map(|m| m.field_value(point)).sum()
}

pub fn compute_metaball_bounds(
    metaballs: &[Metaball],
    threshold: f32,
    initial_pad: f32,
) -> (Vec2, Vec2) {
    compute_metaball_bounds_with_field(metaballs, metaballs, threshold, initial_pad)
}

pub fn compute_metaball_bounds_with_field(
    init_metaballs: &[Metaball],
    field_metaballs: &[Metaball],
    threshold: f32,
    initial_pad: f32,
) -> (Vec2, Vec2) {
    let mut min_x = f32::MAX;
    let mut max_x = f32::MIN;
    let mut min_y = f32::MAX;
    let mut max_y = f32::MIN;
    for m in init_metaballs {
        if m.position.x - m.radius < min_x {
            min_x = m.position.x - m.radius;
        }
        if m.position.x + m.radius > max_x {
            max_x = m.position.x + m.radius;
        }
        if m.position.y - m.radius < min_y {
            min_y = m.position.y - m.radius;
        }
        if m.position.y + m.radius > max_y {
            max_y = m.position.y + m.radius;
        }
    }

    let mut bounds_min = Vec2::new(min_x - initial_pad, min_y - initial_pad);
    let mut bounds_max = Vec2::new(max_x + initial_pad, max_y + initial_pad);

    for _ in 0..20 {
        let sample_below_threshold =
            |p: Vec2| -> bool { sample_field(field_metaballs, p) < threshold };

        let mx = (bounds_min.x + bounds_max.x) * 0.5;
        let my = (bounds_min.y + bounds_max.y) * 0.5;

        let mut all_below = true;

        if !sample_below_threshold(Vec2::new(bounds_min.x, my)) {
            all_below = false;
        }
        if !sample_below_threshold(Vec2::new(bounds_max.x, my)) {
            all_below = false;
        }
        if !sample_below_threshold(Vec2::new(mx, bounds_min.y)) {
            all_below = false;
        }
        if !sample_below_threshold(Vec2::new(mx, bounds_max.y)) {
            all_below = false;
        }

        if !sample_below_threshold(Vec2::new(bounds_min.x, bounds_min.y)) {
            all_below = false;
        }
        if !sample_below_threshold(Vec2::new(bounds_max.x, bounds_min.y)) {
            all_below = false;
        }
        if !sample_below_threshold(Vec2::new(bounds_min.x, bounds_max.y)) {
            all_below = false;
        }
        if !sample_below_threshold(Vec2::new(bounds_max.x, bounds_max.y)) {
            all_below = false;
        }

        if all_below {
            break;
        }

        let expand = 2.0;
        bounds_min.x -= expand;
        bounds_min.y -= expand;
        bounds_max.x += expand;
        bounds_max.y += expand;
    }

    let edge_pad = initial_pad;
    bounds_min.x -= edge_pad;
    bounds_min.y -= edge_pad;
    bounds_max.x += edge_pad;
    bounds_max.y += edge_pad;

    (bounds_min, bounds_max)
}

pub struct MetaballCluster {
    pub metaballs: Vec<Metaball>,
    pub bounds: Aabb,
}

pub fn cluster_metaballs(metaballs: &[Metaball], threshold: f32) -> Vec<MetaballCluster> {
    let n = metaballs.len();
    if n == 0 {
        return Vec::new();
    }

    let mut parent: Vec<usize> = (0..n).collect();

    fn find(parent: &mut [usize], x: usize) -> usize {
        if parent[x] != x {
            parent[x] = find(parent, parent[x]);
        }
        parent[x]
    }

    fn union(parent: &mut [usize], rank: &mut [usize], a: usize, b: usize) {
        let ra = find(parent, a);
        let rb = find(parent, b);
        if ra == rb {
            return;
        }
        if rank[ra] < rank[rb] {
            parent[ra] = rb;
        } else if rank[ra] > rank[rb] {
            parent[rb] = ra;
        } else {
            parent[rb] = ra;
            rank[ra] += 1;
        }
    }

    let mut rank = vec![0usize; n];

    for i in 0..n {
        for j in (i + 1)..n {
            let midpoint = (metaballs[i].position + metaballs[j].position) * 0.5;
            let field_at_mid =
                metaballs[i].field_value(midpoint) + metaballs[j].field_value(midpoint);
            if field_at_mid >= threshold {
                union(&mut parent, &mut rank, i, j);
            }
        }
    }

    let mut groups: std::collections::HashMap<usize, Vec<usize>> = std::collections::HashMap::new();
    for i in 0..n {
        let root = find(&mut parent, i);
        groups.entry(root).or_default().push(i);
    }

    groups
        .into_values()
        .map(|indices| {
            let cluster_balls: Vec<Metaball> = indices.iter().map(|&i| metaballs[i]).collect();
            let mut bounds = Aabb {
                min: cluster_balls[0].position
                    - Vec2::new(cluster_balls[0].radius, cluster_balls[0].radius),
                max: cluster_balls[0].position
                    + Vec2::new(cluster_balls[0].radius, cluster_balls[0].radius),
            };
            for m in &cluster_balls[1..] {
                let m_aabb = Aabb {
                    min: m.position - Vec2::new(m.radius, m.radius),
                    max: m.position + Vec2::new(m.radius, m.radius),
                };
                bounds = bounds.union(&m_aabb);
            }
            MetaballCluster {
                metaballs: cluster_balls,
                bounds,
            }
        })
        .collect()
}

pub struct CellInfo {
    pub i: usize,
    pub j: usize,
    pub bl: f32,
    pub br: f32,
    pub tl: f32,
    pub tr: f32,
    pub case_index: u8,
    pub x0: f32,
    pub y0: f32,
    pub cell_w: f32,
    pub cell_h: f32,
}

pub struct MarchingSquaresDebug {
    pub bounds_min: Vec2,
    pub bounds_max: Vec2,
    pub resolution: usize,
    pub threshold: f32,
    pub grid: Vec<Vec<f32>>,
    pub cell_w: f32,
    pub cell_h: f32,
    pub segments: Vec<(Vec2, Vec2)>,
    pub cells: Vec<CellInfo>,
    pub polygons: Vec<Vec<(f32, f32)>>,
    pub open_chains: Vec<Vec<(f32, f32)>>,
    pub case_histogram: [usize; 16],
    pub grid_min: f32,
    pub grid_max: f32,
}

fn chain_segments(segments: Vec<(Vec2, Vec2)>) -> Vec<Vec<(f32, f32)>> {
    chain_segments_debug(segments).0
}

pub fn marching_squares_debug(
    metaballs: &[Metaball],
    bounds_min: Vec2,
    bounds_max: Vec2,
    resolution: usize,
    threshold: f32,
    obstacles: &[Body],
) -> MarchingSquaresDebug {
    let width = bounds_max.x - bounds_min.x;
    let height = bounds_max.y - bounds_min.y;
    let cell_w = width / resolution as f32;
    let cell_h = height / resolution as f32;

    let mut grid = vec![vec![0.0f32; resolution + 1]; resolution + 1];
    for j in 0..=resolution {
        for i in 0..=resolution {
            let x = bounds_min.x + i as f32 * cell_w;
            let y = bounds_min.y + j as f32 * cell_h;
            grid[j][i] = if obstacles.is_empty() {
                sample_field(metaballs, Vec2::new(x, y))
            } else {
                sample_field_with_obstacles(metaballs, Vec2::new(x, y), obstacles)
            };
        }
    }

    let mut grid_min = f32::MAX;
    let mut grid_max = f32::MIN;
    for j in 0..=resolution {
        for i in 0..=resolution {
            if grid[j][i] < grid_min {
                grid_min = grid[j][i];
            }
            if grid[j][i] > grid_max {
                grid_max = grid[j][i];
            }
        }
    }

    let mut segments: Vec<(Vec2, Vec2)> = Vec::new();
    let mut cells: Vec<CellInfo> = Vec::new();
    let mut case_histogram = [0usize; 16];

    for j in 0..resolution {
        for i in 0..resolution {
            let tl = grid[j + 1][i];
            let tr = grid[j + 1][i + 1];
            let br = grid[j][i + 1];
            let bl = grid[j][i];

            let x0 = bounds_min.x + i as f32 * cell_w;
            let y0 = bounds_min.y + j as f32 * cell_h;

            let mut case_index = 0u8;
            if tl >= threshold {
                case_index |= 8;
            }
            if tr >= threshold {
                case_index |= 4;
            }
            if br >= threshold {
                case_index |= 2;
            }
            if bl >= threshold {
                case_index |= 1;
            }

            case_histogram[case_index as usize] += 1;

            cells.push(CellInfo {
                i,
                j,
                bl,
                br,
                tl,
                tr,
                case_index,
                x0,
                y0,
                cell_w,
                cell_h,
            });

            if case_index == 0 || case_index == 15 {
                continue;
            }

            let x1 = x0 + cell_w;
            let y1 = y0 + cell_h;

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

    let (polygons, open_chains) = chain_segments_debug(segments.clone());

    MarchingSquaresDebug {
        bounds_min,
        bounds_max,
        resolution,
        threshold,
        grid,
        cell_w,
        cell_h,
        segments,
        cells,
        polygons,
        open_chains,
        case_histogram,
        grid_min,
        grid_max,
    }
}

pub fn marching_squares(
    metaballs: &[Metaball],
    bounds_min: Vec2,
    bounds_max: Vec2,
    resolution: usize,
    threshold: f32,
    obstacles: &[Body],
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
            grid[j][i] = if obstacles.is_empty() {
                sample_field(metaballs, Vec2::new(x, y))
            } else {
                sample_field_with_obstacles(metaballs, Vec2::new(x, y), obstacles)
            };
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
            if tl >= threshold {
                case_index |= 8;
            }
            if tr >= threshold {
                case_index |= 4;
            }
            if br >= threshold {
                case_index |= 2;
            }
            if bl >= threshold {
                case_index |= 1;
            }

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

fn chain_segments_debug(
    segments: Vec<(Vec2, Vec2)>,
) -> (Vec<Vec<(f32, f32)>>, Vec<Vec<(f32, f32)>>) {
    if segments.is_empty() {
        return (Vec::new(), Vec::new());
    }

    let eps = 0.001;
    let eq = |a: Vec2, b: Vec2| -> bool { (a.x - b.x).abs() < eps && (a.y - b.y).abs() < eps };

    let mut remaining: Vec<(Vec2, Vec2)> = segments;
    let mut polygons: Vec<Vec<(f32, f32)>> = Vec::new();
    let mut open_chains: Vec<Vec<(f32, f32)>> = Vec::new();

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
        } else {
            open_chains.push(poly);
        }
    }

    (polygons, open_chains)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32, eps: f32) -> bool {
        (a - b).abs() < eps
    }

    #[test]
    fn test_single_metaball_produces_closed_polygon() {
        let metaballs = vec![Metaball::new(Vec2::new(0.0, 0.0), 2.0, 1.0)];
        let bounds_min = Vec2::new(-6.0, -6.0);
        let bounds_max = Vec2::new(6.0, 6.0);
        let debug = marching_squares_debug(&metaballs, bounds_min, bounds_max, 50, 0.5, &[]);

        println!("Single metaball:");
        println!(
            "  Grid range: [{:.3}, {:.3}]",
            debug.grid_min, debug.grid_max
        );
        println!("  Case histogram: {:?}", debug.case_histogram);
        println!("  Segments: {}", debug.segments.len());
        println!("  Closed polygons: {}", debug.polygons.len());
        println!("  Open chains: {}", debug.open_chains.len());
        for (i, poly) in debug.polygons.iter().enumerate() {
            println!("  Polygon {}: {} points", i, poly.len());
        }
        for (i, chain) in debug.open_chains.iter().enumerate() {
            println!(
                "  Open chain {}: {} points, start=({:.3},{:.3}), end=({:.3},{:.3})",
                i,
                chain.len(),
                chain[0].0,
                chain[0].1,
                chain[chain.len() - 1].0,
                chain[chain.len() - 1].1
            );
        }

        assert!(
            !debug.polygons.is_empty(),
            "Should produce at least one closed polygon"
        );
        assert!(debug.open_chains.is_empty(), "Should have no open chains");
    }

    #[test]
    fn test_two_close_metaballs_produce_one_merged_polygon() {
        let metaballs = vec![
            Metaball::new(Vec2::new(-1.0, 0.0), 2.0, 1.0),
            Metaball::new(Vec2::new(1.0, 0.0), 2.0, 1.0),
        ];
        let bounds_min = Vec2::new(-6.0, -6.0);
        let bounds_max = Vec2::new(6.0, 6.0);
        let debug = marching_squares_debug(&metaballs, bounds_min, bounds_max, 50, 0.5, &[]);

        println!("\nTwo close metaballs (merged):");
        println!(
            "  Grid range: [{:.3}, {:.3}]",
            debug.grid_min, debug.grid_max
        );
        println!("  Case histogram: {:?}", debug.case_histogram);
        println!("  Segments: {}", debug.segments.len());
        println!("  Closed polygons: {}", debug.polygons.len());
        println!("  Open chains: {}", debug.open_chains.len());

        let field_at_midpoint = sample_field(&metaballs, Vec2::new(0.0, 0.0));
        println!("  Field at midpoint (0,0): {:.3}", field_at_midpoint);

        assert!(
            !debug.polygons.is_empty(),
            "Should produce at least one closed polygon"
        );
    }

    #[test]
    fn test_two_far_apart_metaballs_produce_two_polygons() {
        let metaballs = vec![
            Metaball::new(Vec2::new(-5.0, 0.0), 2.0, 1.0),
            Metaball::new(Vec2::new(5.0, 0.0), 2.0, 1.0),
        ];
        let bounds_min = Vec2::new(-12.0, -6.0);
        let bounds_max = Vec2::new(12.0, 6.0);
        let debug = marching_squares_debug(&metaballs, bounds_min, bounds_max, 80, 0.5, &[]);

        println!("\nTwo far apart metaballs (separate):");
        println!(
            "  Grid range: [{:.3}, {:.3}]",
            debug.grid_min, debug.grid_max
        );
        println!("  Case histogram: {:?}", debug.case_histogram);
        println!("  Segments: {}", debug.segments.len());
        println!("  Closed polygons: {}", debug.polygons.len());
        println!("  Open chains: {}", debug.open_chains.len());
        for (i, poly) in debug.polygons.iter().enumerate() {
            println!("  Polygon {}: {} points", i, poly.len());
        }

        let field_between = sample_field(&metaballs, Vec2::new(0.0, 0.0));
        println!("  Field between at (0,0): {:.3}", field_between);

        assert!(
            debug.polygons.len() >= 2,
            "Should produce at least two closed polygons, got {}",
            debug.polygons.len()
        );
    }

    #[test]
    fn test_marching_squares_debug_has_no_open_chains() {
        let metaballs = vec![
            Metaball::new(Vec2::new(-1.0, 0.0), 2.0, 1.0),
            Metaball::new(Vec2::new(1.0, 0.0), 2.0, 1.0),
            Metaball::new(Vec2::new(0.0, 1.5), 2.0, 1.0),
        ];
        let bounds_min = Vec2::new(-8.0, -8.0);
        let bounds_max = Vec2::new(8.0, 8.0);
        let debug = marching_squares_debug(&metaballs, bounds_min, bounds_max, 60, 0.5, &[]);

        println!("\nThree metaballs in triangle:");
        println!(
            "  Grid range: [{:.3}, {:.3}]",
            debug.grid_min, debug.grid_max
        );
        println!("  Case histogram: {:?}", debug.case_histogram);
        println!("  Segments: {}", debug.segments.len());
        println!("  Closed polygons: {}", debug.polygons.len());
        println!("  Open chains: {}", debug.open_chains.len());
        for (i, chain) in debug.open_chains.iter().enumerate() {
            println!(
                "  Open chain {}: {} points, start=({:.3},{:.3}), end=({:.3},{:.3})",
                i,
                chain.len(),
                chain[0].0,
                chain[0].1,
                chain[chain.len() - 1].0,
                chain[chain.len() - 1].1
            );
        }

        if !debug.open_chains.is_empty() {
            println!(
                "  WARNING: {} open chains detected!",
                debug.open_chains.len()
            );
            println!("  This indicates the contour reaches the grid boundary");
        }
    }

    #[test]
    fn test_six_circles_on_ground() {
        let spacing = 2.5;
        let radius = 2.0;
        let y = 1.0;
        let metaballs: Vec<Metaball> = (0..6)
            .map(|i| {
                Metaball::new(
                    Vec2::new(-spacing * 2.5 + i as f32 * spacing, y),
                    radius,
                    1.0,
                )
            })
            .collect();

        let (bounds_min, bounds_max) = compute_metaball_bounds(&metaballs, 0.5, 3.0);

        let debug = marching_squares_debug(&metaballs, bounds_min, bounds_max, 50, 0.5, &[]);

        println!("\nSix circles on ground (demo11 scenario):");
        println!(
            "  Bounds: ({:.1},{:.1}) to ({:.1},{:.1})",
            bounds_min.x, bounds_min.y, bounds_max.x, bounds_max.y
        );
        println!(
            "  Grid range: [{:.3}, {:.3}]",
            debug.grid_min, debug.grid_max
        );
        println!("  Case histogram: {:?}", debug.case_histogram);
        println!("  Segments: {}", debug.segments.len());
        println!("  Closed polygons: {}", debug.polygons.len());
        println!("  Open chains: {}", debug.open_chains.len());
        for (i, poly) in debug.polygons.iter().enumerate() {
            println!("  Polygon {}: {} points", i, poly.len());
        }
        for (i, chain) in debug.open_chains.iter().enumerate() {
            println!(
                "  Open chain {}: {} points, start=({:.3},{:.3}), end=({:.3},{:.3})",
                i,
                chain.len(),
                chain[0].0,
                chain[0].1,
                chain[chain.len() - 1].0,
                chain[chain.len() - 1].1
            );
        }

        let field_at_center = sample_field(&metaballs, Vec2::new(0.0, y));
        let field_at_left = sample_field(&metaballs, Vec2::new(-6.25, y));
        let field_between = sample_field(&metaballs, Vec2::new(-3.125, y));
        println!("  Field at center (0,{:.1}): {:.3}", y, field_at_center);
        println!("  Field at leftmost (-6.25,{:.1}): {:.3}", y, field_at_left);
        println!(
            "  Field between circles (-3.125,{:.1}): {:.3}",
            y, field_between
        );

        if !debug.open_chains.is_empty() {
            println!(
                "  WARNING: {} open chains detected!",
                debug.open_chains.len()
            );
        }
    }
}
