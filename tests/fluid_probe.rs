//! Headless probe to find the best fluid-particle configuration.
//! Replicates the pinball example's fluid mesh, drops a ball onto it, and
//! measures two competing goals independently:
//!   * PASS  — the ball must NOT be blocked (how deep it reaches).
//!   * JOSTLE— the particles must realistically part/slosh on contact.
//!
//! Run with: cargo test --test fluid_probe -- --nocapture

use sylt_2d::body::{Body, Shape};
use sylt_2d::joint::Joint;
use sylt_2d::math_utils::Vec2;
use sylt_2d::world::World;

const ITER: u32 = 120;
const DT: f32 = 1.0 / 60.0;
const BALL_R: f32 = 0.35;

struct Config {
    name: &'static str,
    /// Sensor particles collide only with each other (liquid jostle) and let
    /// the ball pass through (parted by `displace`). Otherwise they are rigid.
    sensor: bool,
    /// Manual displacement force pushed onto neighbours touching the ball
    /// (only used for sensor configs). 0 = none.
    displace: f32,
    particle_r: f32,
    spacing: f32,
    particle_mass: f32,
    link_softness: f32,
    anchor_softness: f32,
    hub_freq_hz: f32,
    hub_damping: f32,
    link: bool,
}

const CONFIGS: &[Config] = &[
    Config {
        name: "A sensor/current",
        sensor: true,
        displace: 30.0,
        particle_r: 0.16,
        spacing: 0.5,
        particle_mass: 0.35,
        link_softness: 0.05,
        anchor_softness: 0.08,
        hub_freq_hz: 3.0,
        hub_damping: 0.45,
        link: true,
    },
    Config {
        name: "B rigid(blocked)",
        sensor: false,
        displace: 0.0,
        particle_r: 0.16,
        spacing: 0.5,
        particle_mass: 0.35,
        link_softness: 0.05,
        anchor_softness: 0.08,
        hub_freq_hz: 3.0,
        hub_damping: 0.45,
        link: true,
    },
    Config {
        name: "C rigid light+loose",
        sensor: false,
        displace: 0.0,
        particle_r: 0.14,
        spacing: 0.7,
        particle_mass: 0.08,
        link_softness: 0.18,
        anchor_softness: 0.25,
        hub_freq_hz: 2.0,
        hub_damping: 0.3,
        link: true,
    },
    Config {
        name: "D sensor/loose+react",
        sensor: true,
        displace: 60.0,
        particle_r: 0.14,
        spacing: 0.6,
        particle_mass: 0.05,
        link_softness: 0.35,
        anchor_softness: 0.4,
        hub_freq_hz: 2.0,
        hub_damping: 0.25,
        link: false,
    },
    Config {
        name: "E rigid/rubble",
        sensor: false,
        displace: 0.0,
        particle_r: 0.12,
        spacing: 0.75,
        particle_mass: 0.03,
        link_softness: 0.4,
        anchor_softness: 0.4,
        hub_freq_hz: 1.5,
        hub_damping: 0.2,
        link: false,
    },
    Config {
        name: "F rigid/dense light",
        sensor: false,
        displace: 0.0,
        particle_r: 0.11,
        spacing: 0.5,
        particle_mass: 0.02,
        link_softness: 0.4,
        anchor_softness: 0.45,
        hub_freq_hz: 1.5,
        hub_damping: 0.2,
        link: false,
    },
    Config {
        name: "G sensor/dense react",
        sensor: true,
        displace: 40.0,
        particle_r: 0.13,
        spacing: 0.5,
        particle_mass: 0.04,
        link_softness: 0.4,
        anchor_softness: 0.45,
        hub_freq_hz: 1.5,
        hub_damping: 0.2,
        link: false,
    },
];

struct Report {
    name: &'static str,
    ball_min_y: f32,
    final_y: f32,
    jostle: f32,
    stretch: f32,
    cohesion: f32,
    pass: bool,
}

fn spawn_pool(world: &mut World, center: Vec2, radius: f32, cfg: &Config) {
    // Static centre hub holds the blob in place (like the example). It is a
    // sensor so it never blocks the pinball itself.
    let mut hub = Body::new(Vec2::new(0.05, 0.05), f32::MAX);
    hub.position = center;
    hub.set_sensor(true);
    world.add_body(hub.clone());

    let t = 1.0 / 60.0;
    let omega = 2.0 * std::f32::consts::PI * cfg.hub_freq_hz;
    let d = 2.0 * cfg.particle_mass * cfg.hub_damping * omega;
    let k = cfg.particle_mass * omega * omega;
    let bias = t * k / (d + t * k);

    let mut particles: Vec<Body> = Vec::new();
    let mut x = -radius;
    let mut row = 0i32;
    while x <= radius {
        let y_off = if row % 2 == 0 { 0.0 } else { cfg.spacing * 0.5 };
        let mut y = -radius;
        while y <= radius {
            let p = Vec2::new(x, y + y_off);
            if p.length() <= radius {
                let mut b = Body::new_circle(cfg.particle_r, cfg.particle_mass);
                b.position = center + p;
                b.friction = 0.05;
                b.set_sensor(cfg.sensor);
                world.add_body(b.clone());
                particles.push(b);
            }
            y += cfg.spacing;
        }
        x += cfg.spacing * 0.866;
        row += 1;
    }

    if cfg.link {
        let link_thresh = cfg.spacing * 1.5;
        for i in 0..particles.len() {
            for j in (i + 1)..particles.len() {
                let a = particles[i].position;
                let b = particles[j].position;
                if (b - a).length() < link_thresh {
                    let mut jj = Joint::new(
                        particles[i].clone(),
                        particles[j].clone(),
                        (a + b) * 0.5,
                        world,
                    );
                    jj.softness = cfg.link_softness;
                    jj.bias_factor = bias;
                    world.add_joint(jj);
                }
            }
        }
    }
    for p in &particles {
        let mut jj = Joint::new(hub.clone(), p.clone(), (p.position + center) * 0.5, world);
        jj.softness = cfg.anchor_softness;
        jj.bias_factor = bias;
        world.add_joint(jj);
    }
}

fn probe_run(cfg: &Config) -> Report {
    let mut world = World::new(Vec2::new(0.0, -18.0), ITER);
    let center = Vec2::new(0.0, 0.0);
    let pool_r = 1.5;

    spawn_pool(&mut world, center, pool_r, cfg);

    // Static floor so the ball settles and we can read where it ends up.
    let mut floor = Body::new(Vec2::new(20.0, 0.4), f32::MAX);
    floor.position = Vec2::new(0.0, -3.5);
    world.add_body(floor);

    let particle_ids: Vec<usize> = world
        .bodies
        .iter()
        .filter(|b| b.borrow().shape == Shape::Circle && b.borrow().inv_mass != 0.0)
        .map(|b| b.borrow().id)
        .collect();
    let rest_pos: Vec<Vec2> = particle_ids
        .iter()
        .map(|&i| {
            world
                .bodies
                .iter()
                .find(|b| b.borrow().id == i)
                .unwrap()
                .borrow()
                .position
        })
        .collect();

    // Ball just above the pool, dropped in from rest.
    let mut ball = Body::new_circle(BALL_R, 1.5);
    ball.position = Vec2::new(0.0, center.y + pool_r + BALL_R + 0.3);
    ball.friction = 0.02;
    world.add_body(ball.clone());
    let ball_id = ball.id;

    let mut min_y = f32::INFINITY;
    let mut final_y = 0f32;
    let mut jostle = 0f32;
    let mut stretch = 0f32;

    const STEPS: usize = 250;
    for _ in 0..STEPS {
        let _ = world.step(DT);

        // liquid drag (like apply_fluid_drag)
        let bpos = {
            world
                .bodies
                .iter()
                .find(|b| b.borrow().id == ball_id)
                .unwrap()
                .borrow()
                .position
        };
        if (bpos - center).length() < pool_r {
            let mut ball_b = world
                .bodies
                .iter()
                .find(|b| b.borrow().id == ball_id)
                .unwrap()
                .borrow_mut();
            let v = ball_b.velocity;
            ball_b.velocity = v + v * (-2.0) * DT;
            drop(ball_b);
        }

        // manual pump displacement for sensor configs (mirrors the example).
        if cfg.sensor {
            for &id in &particle_ids {
                let mut pb = world
                    .bodies
                    .iter()
                    .find(|b| b.borrow().id == id)
                    .unwrap()
                    .borrow_mut();
                let d = pb.position - bpos;
                let dist = d.length();
                let r_sum = BALL_R + cfg.particle_r + 0.15;
                if dist > f32::EPSILON && dist < r_sum {
                    let n = d * (1.0 / dist);
                    pb.add_force(n * (r_sum - dist) * cfg.displace);
                }
            }
        }

        let by = world
            .bodies
            .iter()
            .find(|b| b.borrow().id == ball_id)
            .unwrap()
            .borrow()
            .position
            .y;
        if by < min_y {
            min_y = by;
        }

        let mut sum_move = 0f32;
        let mut max_stretch = 0f32;
        for (idx, &id) in particle_ids.iter().enumerate() {
            let p = world
                .bodies
                .iter()
                .find(|b| b.borrow().id == id)
                .unwrap()
                .borrow()
                .position;
            sum_move += (p - rest_pos[idx]).length();
            let dc = (p - center).length() - cfg.particle_r;
            if dc > max_stretch {
                max_stretch = dc;
            }
        }
        let avg = sum_move / particle_ids.len().max(1) as f32;
        if avg > jostle {
            jostle = avg;
        }
        if max_stretch > stretch {
            stretch = max_stretch;
        }
    }

    let mut cohesion = 0f32;
    for &id in &particle_ids {
        let p = world
            .bodies
            .iter()
            .find(|b| b.borrow().id == id)
            .unwrap()
            .borrow()
            .position;
        cohesion += (p - center).length();
    }
    cohesion /= particle_ids.len().max(1) as f32;

    final_y = world
        .bodies
        .iter()
        .find(|b| b.borrow().id == ball_id)
        .unwrap()
        .borrow()
        .position
        .y;

    Report {
        name: cfg.name,
        ball_min_y: min_y,
        final_y,
        jostle,
        stretch,
        cohesion,
        pass: min_y < center.y,
    }
}

#[test]
fn fluid_probe_prints_table() {
    println!("\nfluid-probe: pool centre y=0 radius=1.5, ball starts on top");
    println!("  minBallY: lowest the ball reached (more negative = carved deeper through)");
    println!("  jostle  : avg how far particles moved from rest (slosh amount)");
    println!("  stretch : how far the blob outline got pushed/carved");
    println!("  cohesion: avg dist particles keep from centre at end (blob holds together)\n");
    println!(
        "{:<20} {:>9} {:>8} {:>8} {:>8} {:>8}   verdict",
        "config", "minBallY", "finalY", "jostle", "stretch", "cohesion"
    );
    for cfg in CONFIGS {
        let r = probe_run(cfg);
        println!(
            "{:<20} {:>9.2} {:>8.2} {:>8.2} {:>8.2} {:>8.2}   {}",
            r.name,
            r.ball_min_y,
            r.final_y,
            r.jostle,
            r.stretch,
            r.cohesion,
            if r.pass { "PASSES" } else { "BLOCKED" }
        );
    }
}
