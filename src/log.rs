use crate::arbiter::ArbiterLog;
use crate::body::Body;
use crate::joint::JointLog;
use crate::math_utils::Vec2;
use crate::world::WorldContext;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;

#[cfg(feature = "log")]
#[derive(Debug, Clone, serde::Serialize)]
pub struct StepLog {
    pub step: u64,
    pub dt: f32,
    pub inv_dt: f32,
    pub gravity: Vec2,
    pub context: ContextLog,
    pub bodies: Vec<BodyLog>,
    pub arbiters: Vec<ArbiterLog>,
    pub joints: Vec<JointLog>,
}

#[cfg(feature = "log")]
#[derive(Debug, Clone, serde::Serialize)]
pub struct ContextLog {
    pub accumulate_impulse: bool,
    pub warm_starting: bool,
    pub position_correction: bool,
}

#[cfg(feature = "log")]
#[derive(Debug, Clone, serde::Serialize)]
pub struct BodyLog {
    pub id: usize,
    pub shape: String,
    pub position: Vec2,
    pub rotation: f32,
    pub velocity: Vec2,
    pub angular_velocity: f32,
    pub force: Vec2,
    pub torque: f32,
    pub mass: f32,
    pub inv_mass: f32,
    pub moi: f32,
    pub radius: f32,
    pub is_static: bool,
}

pub struct Logger {
    writer: BufWriter<File>,
    step_counter: u64,
}

impl Logger {
    pub fn new(path: impl Into<PathBuf>) -> Self {
        let path = path.into();
        let file = File::create(&path)
            .unwrap_or_else(|e| panic!("Failed to create log file {:?}: {}", path, e));
        let writer = BufWriter::new(file);
        Self {
            writer,
            step_counter: 0,
        }
    }

    pub fn log_step(
        &mut self,
        dt: f32,
        gravity: Vec2,
        world_context: &WorldContext,
        bodies: &[std::rc::Rc<std::cell::RefCell<Body>>],
        arbiters: &[ArbiterLog],
        joints: &[JointLog],
    ) {
        let inv_dt = if dt > 0.0 { 1.0 / dt } else { 0.0 };

        let body_logs: Vec<BodyLog> = bodies
            .iter()
            .map(|b| {
                let b = b.borrow();
                BodyLog {
                    id: b.id,
                    shape: format!("{:?}", b.shape),
                    position: b.position,
                    rotation: b.rotation,
                    velocity: b.velocity,
                    angular_velocity: b.angular_velocity,
                    force: b.force,
                    torque: b.torque,
                    mass: b.mass,
                    inv_mass: b.inv_mass,
                    moi: b.moi,
                    radius: b.radius,
                    is_static: b.inv_mass == 0.0,
                }
            })
            .collect();

        let step_log = StepLog {
            step: self.step_counter,
            dt,
            inv_dt,
            gravity,
            context: ContextLog {
                accumulate_impulse: world_context.accumulate_impulse,
                warm_starting: world_context.warm_starting,
                position_correction: world_context.position_correction,
            },
            bodies: body_logs,
            arbiters: arbiters.to_vec(),
            joints: joints.to_vec(),
        };

        if let Ok(mut json) = serde_json::to_string(&step_log) {
            json.push('\n');
            let _ = self.writer.write_all(json.as_bytes());
            let _ = self.writer.flush();
        }

        self.step_counter += 1;
    }
}
