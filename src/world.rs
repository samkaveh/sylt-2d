use crate::arbiter::{Arbiter, ArbiterKey};
use crate::body::{Body, Shape};
use crate::errors::Sylt2DErrors;
use crate::joint::Joint;
use crate::math_utils::{Aabb, Vec2};
use crate::sweep::{sweep_box_box, sweep_circle_circle, sweep_circle_polygon, Impact};
use std::cell::{Ref, RefCell};
use std::collections::HashMap;
use std::rc::Rc;
use std::slice::Iter;

#[cfg(feature = "log")]
use crate::log::Logger;

#[derive(Clone, Copy)]
pub struct WorldContext {
    pub accumulate_impulse: bool,
    pub warm_starting: bool,
    pub position_correction: bool,
    /// Restitution (bounciness) applied at contacts. 0.0 = no bounce (default),
    /// values in [0,1] control how much kinetic energy is retained on the
    /// normal direction (e.g. 0.2 makes balls bounce).
    pub restitution: f32,
}

#[derive(Clone, Copy)]
struct SapEndpoint {
    value: f32,
    body_index: usize,
    is_max: bool,
}

pub struct World {
    gravity: Vec2,
    iterations: u32,
    pub world_context: WorldContext,
    pub bodies: Vec<Rc<RefCell<Body>>>,
    pub joints: Vec<Joint>,
    pub arbiters: HashMap<ArbiterKey, Arbiter>,
    sap_endpoints: Vec<SapEndpoint>,
    sap_aabbs: Vec<Aabb>,
    /// Continuous collision detection: when true, fast-moving dynamic bodies
    /// are swept against potential colliders and their motion is clamped to the
    /// point of impact, preventing tunneling.
    ccd_enabled: bool,
    /// Speed above which a body is treated as a "bullet" for CCD.
    ccd_speed_threshold: f32,
    #[cfg(feature = "log")]
    pub logger: Option<Logger>,
}

pub struct BodiesIter<'a> {
    inner: Iter<'a, Rc<RefCell<Body>>>,
}
impl<'a> Iterator for BodiesIter<'a> {
    type Item = Ref<'a, Body>;
    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next().map(|body| body.borrow())
    }
}

impl World {
    pub fn new(gravity: Vec2, iterations: u32) -> Self {
        let context = WorldContext {
            accumulate_impulse: true,
            warm_starting: false,
            position_correction: true,
            restitution: 0.0,
        };
        Self {
            gravity,
            iterations,
            world_context: context,
            bodies: Vec::<Rc<RefCell<Body>>>::with_capacity(2),
            joints: Vec::<Joint>::with_capacity(2),
            arbiters: HashMap::<ArbiterKey, Arbiter>::new(),
            sap_endpoints: Vec::new(),
            sap_aabbs: Vec::new(),
            ccd_enabled: true,
            ccd_speed_threshold: 6.0,
            #[cfg(feature = "log")]
            logger: None,
        }
    }

    pub fn add_body(&mut self, body: Body) {
        self.bodies.push(Rc::new(RefCell::new(body)));
    }

    pub fn iter_bodies(&self) -> BodiesIter {
        BodiesIter {
            inner: self.bodies.iter(),
        }
    }

    pub fn add_joint(&mut self, joint: Joint) {
        self.joints.push(joint);
    }

    pub fn clear(&mut self) {
        self.bodies.clear();
        self.joints.clear();
        self.arbiters.clear();
        self.sap_endpoints.clear();
        self.sap_aabbs.clear();
    }

    pub fn set_ccd_enabled(&mut self, enabled: bool) {
        self.ccd_enabled = enabled;
    }

    /// True if two bodies (by id) are linked by a joint and must not collide.
    fn pair_is_jointed(&self, a: usize, b: usize) -> bool {
        self.joints.iter().any(|j| {
            let id1 = j.body_1.borrow().id;
            let id2 = j.body_2.borrow().id;
            (id1 == a && id2 == b) || (id1 == b && id2 == a)
        })
    }

    pub fn set_ccd_speed_threshold(&mut self, threshold: f32) {
        self.ccd_speed_threshold = threshold;
    }

    pub fn set_restitution(&mut self, e: f32) {
        self.world_context.restitution = e.clamp(0.0, 1.0);
    }

    #[cfg(feature = "log")]
    pub fn set_logger(&mut self, logger: Logger) {
        self.logger = Some(logger);
    }

    fn sort_endpoints(endpoints: &mut [SapEndpoint]) {
        let len = endpoints.len();
        for i in 1..len {
            let key = endpoints[i];
            let mut j = i;
            while j > 0 && endpoints[j - 1].value > key.value {
                endpoints[j] = endpoints[j - 1];
                j -= 1;
            }
            endpoints[j] = key;
        }
    }

    /// Fat AABB margin for broad-phase candidate generation. Bodies whose AABBs
    /// only graze each other (e.g. an exactly-touching contact lands on a float
    /// rounding gap) must still be reported so the narrow phase can build the
    /// manifold.
    const AABB_MARGIN: f32 = 0.01;

    pub fn broad_phase(&mut self) -> Result<(), Sylt2DErrors> {
        let n = self.bodies.len();

        self.sap_aabbs.clear();
        self.sap_aabbs.reserve(n);
        for body in &self.bodies {
            self.sap_aabbs
                .push(body.borrow().get_aabb().expand(Self::AABB_MARGIN));
        }

        self.sap_endpoints.clear();
        self.sap_endpoints.reserve(n * 2);
        for i in 0..n {
            let aabb = self.sap_aabbs[i];
            self.sap_endpoints.push(SapEndpoint {
                value: aabb.min.x,
                body_index: i,
                is_max: false,
            });
            self.sap_endpoints.push(SapEndpoint {
                value: aabb.max.x,
                body_index: i,
                is_max: true,
            });
        }

        Self::sort_endpoints(&mut self.sap_endpoints);

        let mut active: Vec<usize> = Vec::with_capacity(16);
        let mut candidate_pairs: Vec<(usize, usize)> = Vec::with_capacity(32);

        for ep in &self.sap_endpoints {
            if ep.is_max {
                active.retain(|&idx| idx != ep.body_index);
            } else {
                let aabb_i = self.sap_aabbs[ep.body_index];
                for &active_idx in &active {
                    let aabb_j = self.sap_aabbs[active_idx];
                    if aabb_i.overlaps(&aabb_j) {
                        let (lo, hi) = if ep.body_index < active_idx {
                            (ep.body_index, active_idx)
                        } else {
                            (active_idx, ep.body_index)
                        };
                        candidate_pairs.push((lo, hi));
                    }
                }
                active.push(ep.body_index);
            }
        }

        let mut pairs_to_remove: Vec<ArbiterKey> = Vec::new();
        for key in self.arbiters.keys() {
            let found = candidate_pairs.iter().any(|&(a, b)| key.matches(a, b));
            if !found {
                pairs_to_remove.push(ArbiterKey::new_by_id(key.body1_id(), key.body2_id()));
            }
        }
        for key in pairs_to_remove {
            self.arbiters.remove(&key);
        }

        for (i, j) in candidate_pairs {
            let body_i = &self.bodies[i];
            let body_j = &self.bodies[j];
            {
                let bi = body_i.borrow();
                let bj = body_j.borrow();
                if bi.inv_mass == 0.0 && bj.inv_mass == 0.0 {
                    continue;
                }
            }
            // Bodies linked by a joint (e.g. a flipper and its fixed pivot
            // anchor) must never collide with each other. Their anchors overlap
            // by construction, and letting the contact solver push them apart
            // applies a bogus torque that makes the flipper creep/drift.
            let id_i = body_i.borrow().id;
            let id_j = body_j.borrow().id;
            if self.pair_is_jointed(id_i, id_j) {
                continue;
            }

            let key = ArbiterKey::new(&body_i.borrow(), &body_j.borrow());
            let new_arbiter = Arbiter::new(body_i.clone(), body_j.clone());

            if new_arbiter.num_contacts > 0 {
                match self.arbiters.entry(key) {
                    std::collections::hash_map::Entry::Occupied(mut entry) => {
                        let arbiter = entry.get_mut();
                        arbiter.update(
                            new_arbiter.contacts.as_ref(),
                            new_arbiter.num_contacts,
                            &self.world_context,
                        )?
                    }
                    std::collections::hash_map::Entry::Vacant(entry) => {
                        entry.insert(new_arbiter);
                    }
                }
            } else {
                self.arbiters.remove(&key);
            }
        }
        Ok(())
    }

    pub fn step(&mut self, dt: f32) -> Result<(), Sylt2DErrors> {
        let inv_dt = if dt > 0.0 { 1.0 / dt } else { 0.0 };

        // Integrate forces.
        for body in self.bodies.iter() {
            let mut body = body.borrow_mut();
            if body.inv_mass == 0.0 {
                continue;
            }
            body.velocity = body.velocity + (self.gravity + body.force * body.inv_mass) * dt;
            body.angular_velocity += body.inv_moi * body.torque * dt;
        }

        if self.ccd_enabled && dt > 0.0 {
            self.broad_phase_ccd(dt)?;
        } else {
            // Determine overlapping bodies and update contact points.
            self.broad_phase()?;
            self.solve(dt, inv_dt)?;
        }

        #[cfg(feature = "log")]
        if let Some(ref mut logger) = self.logger {
            let arbiters_log: Vec<_> = self.arbiters.values().map(|a| a.to_log()).collect();
            let joints_log: Vec<_> = self.joints.iter().map(|j| j.to_log()).collect();
            logger.log_step(
                dt,
                self.gravity,
                &self.world_context,
                &self.bodies,
                &arbiters_log,
                &joints_log,
            );
        }

        Ok(())
    }

    /// Runs one solve pass: broadphase (rebuild from current positions), pre-step
    /// of arbiters and joints, then impulse iterations, then position integration.
    fn solve(&mut self, dt: f32, inv_dt: f32) -> Result<(), Sylt2DErrors> {
        self.solve_constraints(inv_dt)?;
        self.integrate_positions(dt);
        Ok(())
    }

    /// Resolves contacts/joints for the current configuration WITHOUT integrating
    /// positions (used by the CCD impact sub-step).
    fn solve_constraints(&mut self, inv_dt: f32) -> Result<(), Sylt2DErrors> {
        self.broad_phase()?;

        // Pre-steps
        for (_, arbiter) in self.arbiters.iter_mut() {
            arbiter.pre_step(inv_dt, &self.world_context);
        }

        for joint in self.joints.iter_mut() {
            joint.pre_step(&self.world_context, inv_dt)?;
        }

        // Iterations
        for _ in 0..self.iterations {
            for (_, arbiter) in self.arbiters.iter_mut() {
                arbiter.apply_impulse(&self.world_context);
            }

            for joint in self.joints.iter_mut() {
                joint.apply_impulse();
            }
        }

        Ok(())
    }

    /// Continuous collision detection pass. Fast-moving dynamic bodies are swept
    /// over the remaining time of the frame. When an impact is found we advance
    /// positions to just before that instant, resolve constraints there (which
    /// reflects the fast body's velocity), and then recurse on the leftover time
    /// so every impact within the frame is handled — not just the earliest one.
    fn broad_phase_ccd(&mut self, dt: f32) -> Result<(), Sylt2DErrors> {
        // Bias/position-correction uses the frame's effective timestep so that
        // small sub-steps don't produce huge corrective velocities.
        let bias_inv_dt = if dt > 0.0 { 1.0 / dt } else { 0.0 };

        let mut remaining = dt;
        let mut guard = 0;
        while remaining > 1e-6 && guard < 64 {
            guard += 1;

            let toi = self.find_ccd_toi(remaining);
            match toi {
                // No fast body will tunnel in the leftover time; let the discrete
                // solver finish this slice.
                None => {
                    self.solve(remaining, bias_inv_dt)?;
                    return Ok(());
                }
                Some(impact) => {
                    let partial = impact.t * remaining;

                    // Impact essentially at t=0 → already touching/penetrating.
                    // The discrete solver handles the whole remaining slice.
                    if partial < 1e-6 {
                        self.solve(remaining, bias_inv_dt)?;
                        return Ok(());
                    }

                    // Advance to the impact, reflect velocities there, then keep
                    // sweeping the leftover time for the next impact.
                    self.integrate_positions(partial);
                    self.solve_constraints(bias_inv_dt)?;
                    remaining -= partial;
                }
            }
        }

        // Guard safety net: if the loop exhausted its budget, finish remaining.
        if remaining > 1e-6 {
            self.solve(remaining, bias_inv_dt)?;
        }
        Ok(())
    }

    /// Integrates only positions (using current velocities) for `sub_dt`.
    fn integrate_positions(&self, sub_dt: f32) {
        for body in self.bodies.iter() {
            let mut body = body.borrow_mut();
            body.position = body.position + body.velocity * sub_dt;
            body.rotation += body.angular_velocity * sub_dt;

            body.force = Vec2::default();
            body.torque = 0.0;
        }
    }

    /// Search for the earliest time-of-impact among fast-moving dynamic bodies.
    fn find_ccd_toi(&self, dt: f32) -> Option<Impact> {
        let n = self.bodies.len();
        if n == 0 {
            return None;
        }
        let mut best: Option<Impact> = None;

        for i in 0..n {
            let i_is_fast = {
                let bi = self.bodies[i].borrow();
                // A body counts as a CCD bullet if the speed of its *fastest
                // surface point* exceeds the threshold. Rotation matters: a
                // flipper pivoting in place has ~zero linear velocity but its
                // tip sweeps an arc that can outrun a frame.
                let lin_speed = bi.velocity.length();
                let spin_speed = bi.angular_velocity.abs() * bi.max_edge_speed_radius();
                bi.inv_mass != 0.0 && lin_speed + spin_speed >= self.ccd_speed_threshold
            };
            if !i_is_fast {
                continue;
            }

            for j in 0..n {
                if j == i {
                    continue;
                }
                if self.pair_is_jointed(self.bodies[i].borrow().id, self.bodies[j].borrow().id) {
                    continue;
                }

                let impact = {
                    let bi = self.bodies[i].borrow();
                    let bj = self.bodies[j].borrow();
                    self.sweep_pair(&*bi, &*bj, dt)
                };
                if let Some(impact) = impact {
                    let t_clamped = impact.t.clamp(0.0, 1.0);
                    match &best {
                        None => best = Some(impact),
                        Some(ref cur) if t_clamped < cur.t => best = Some(impact),
                        _ => {}
                    }
                }
            }
        }

        best
    }

    /// Dispatches a sweep test for a pair of bodies given their current state.
    fn sweep_pair(&self, a: &Body, b: &Body, dt: f32) -> Option<Impact> {
        match (a.shape, b.shape) {
            (Shape::Circle, Shape::Circle) => sweep_circle_circle(a, b, dt),
            (Shape::Circle, Shape::Box) | (Shape::Circle, Shape::ConvexPolygon) => {
                sweep_circle_polygon(a, b, dt)
            }
            (Shape::Box, Shape::Circle) | (Shape::ConvexPolygon, Shape::Circle) => {
                sweep_circle_polygon(b, a, dt)
            }
            (Shape::Box, Shape::Box)
            | (Shape::Box, Shape::ConvexPolygon)
            | (Shape::ConvexPolygon, Shape::Box)
            | (Shape::ConvexPolygon, Shape::ConvexPolygon) => sweep_box_box(a, b, dt),
        }
    }
}
