use crate::arbiter::{Arbiter, ArbiterKey};
use crate::body::Body;
use crate::errors::Sylt2DErrors;
use crate::joint::Joint;
use crate::math_utils::{Aabb, Vec2};
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

    pub fn broad_phase(&mut self) -> Result<(), Sylt2DErrors> {
        let n = self.bodies.len();

        self.sap_aabbs.clear();
        self.sap_aabbs.reserve(n);
        for body in &self.bodies {
            self.sap_aabbs.push(body.borrow().get_aabb());
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
            let found = candidate_pairs
                .iter()
                .any(|&(a, b)| key.matches(a, b));
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

            let new_arbiter = Arbiter::new(body_i.clone(), body_j.clone());
            let key = ArbiterKey::new(&body_i.borrow(), &body_j.borrow());

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
        // Determine overlapping bodies and update contact points.
        self.broad_phase()?;

        // Integrate forces.
        for body in self.bodies.iter() {
            let mut body = body.borrow_mut();
            if body.inv_mass == 0.0 {
                continue;
            };
            body.velocity = body.velocity + (self.gravity + body.force * body.inv_mass) * dt;
            body.angular_velocity += body.inv_moi * body.torque * dt;
        }

        // Pefrom pre-steps
        for (_, arbiter) in self.arbiters.iter_mut() {
            arbiter.pre_step(inv_dt, &self.world_context);
        }

        for joint in self.joints.iter_mut() {
            joint.pre_step(&self.world_context, inv_dt)?;
        }

        // Perfrom iterations
        for _ in 0..self.iterations {
            for (_, arbiter) in self.arbiters.iter_mut() {
                arbiter.apply_impulse(&self.world_context);
            }

            for joint in self.joints.iter_mut() {
                joint.apply_impulse();
            }
        }

        // Integrate Velocities
        for body in self.bodies.iter() {
            let mut body = body.borrow_mut();
            body.position = body.position + body.velocity * dt;
            body.rotation += body.angular_velocity * dt;

            body.force = Vec2::default();
            body.torque = 0.0;
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
}
