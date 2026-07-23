use crate::errors::Sylt2DErrors;
use crate::{
    body::Body,
    math_utils::{Cross, Mat2x2, Vec2},
    world::{World, WorldContext},
};
use std::cell::RefCell;
use std::rc::Rc;

#[derive(Default)]
pub struct Joint {
    p: Vec2, // accumulated impuls
    bias: Vec2,
    r1: Vec2,
    r2: Vec2,
    m: Mat2x2,
    pub bias_factor: f32,
    pub softness: f32,
    pub local_anchor_1: Vec2,
    pub local_anchor_2: Vec2,
    pub body_1: Rc<RefCell<Body>>,
    pub body_2: Rc<RefCell<Body>>,
}

impl Joint {
    pub fn new(body_1: Body, body_2: Body, anchor: Vec2, world: &World) -> Self {
        let body_1_rc = world
            .bodies
            .iter()
            .find(|body| body.borrow().id == body_1.id)
            .expect("couldn't find body 1 in world bodies.");
        let body_2_rc = world
            .bodies
            .iter()
            .find(|body| body.borrow().id == body_2.id)
            .expect("couldn't find body 2 in world bodies.");
        let rot_trans_1 = Mat2x2::new_from_angle(body_1_rc.borrow().rotation).transpose();
        let rot_trans_2 = Mat2x2::new_from_angle(body_2_rc.borrow().rotation).transpose();
        let local_anchor_1 = rot_trans_1 * (anchor - body_1_rc.borrow().position);
        let local_anchor_2 = rot_trans_2 * (anchor - body_2_rc.borrow().position);

        Self {
            body_1: body_1_rc.clone(),
            body_2: body_2_rc.clone(),
            local_anchor_1,
            local_anchor_2,
            softness: 0.0,
            bias_factor: 0.2,
            bias: Vec2::new(0.0, 0.0),
            p: Vec2::new(0.0, 0.0),
            r1: Vec2::new(0.0, 0.0),
            r2: Vec2::new(0.0, 0.0),
            m: Mat2x2::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0)),
        }
    }

    pub fn pre_step(
        &mut self,
        world_context: &WorldContext,
        inv_dt: f32,
    ) -> Result<(), Sylt2DErrors> {
        let mut body_1 = self.body_1.borrow_mut();
        let mut body_2 = self.body_2.borrow_mut();
        let rot_1 = Mat2x2::new_from_angle(body_1.rotation);
        let rot_2 = Mat2x2::new_from_angle(body_2.rotation);

        self.r1 = rot_1 * self.local_anchor_1;
        self.r2 = rot_2 * self.local_anchor_2;

        // deltaV = deltaV0 + K * impulse
        // invM = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
        //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
        let mut k1 = Mat2x2::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0));
        k1.col1.x = body_1.inv_mass + body_2.inv_mass;
        k1.col2.x = 0.0;
        k1.col1.y = 0.0;
        k1.col2.y = body_1.inv_mass + body_2.inv_mass;

        let mut k2 = Mat2x2::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0));
        k2.col1.x = body_1.inv_moi * self.r1.y * self.r1.y;
        k2.col2.x = -body_1.inv_moi * self.r1.x * self.r1.y;
        k2.col1.y = -body_1.inv_moi * self.r1.x * self.r1.y;
        k2.col2.y = body_1.inv_moi * self.r1.x * self.r1.x;

        let mut k3 = Mat2x2::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0));
        k3.col1.x = body_2.inv_moi * self.r2.y * self.r2.y;
        k3.col2.x = -body_2.inv_moi * self.r2.x * self.r2.y;
        k3.col1.y = -body_2.inv_moi * self.r2.x * self.r2.y;
        k3.col2.y = body_2.inv_moi * self.r2.x * self.r2.x;

        let mut k = k1 + k2 + k3;
        k.col1.x += self.softness;
        k.col2.y += self.softness;
        self.m = k.invert()?;
        let p1 = body_1.position + self.r1;
        let p2 = body_2.position + self.r2;
        let dp = p2 - p1;

        if world_context.position_correction {
            self.bias = dp * inv_dt * self.bias_factor * -1.0;
        } else {
            self.bias = Vec2::new(0.0, 0.0);
        }

        if world_context.warm_starting {
            body_1.velocity = body_1.velocity - self.p * body_1.inv_mass;
            body_1.angular_velocity -= body_1.inv_moi * self.r1.cross(self.p);
            body_2.velocity = body_2.velocity + self.p * body_2.inv_mass;
            body_2.angular_velocity += body_2.inv_moi * self.r2.cross(self.p);
        } else {
            self.p = Vec2::new(0.0, 0.0);
        }
        Ok(())
    }
    pub fn apply_impulse(&mut self) {
        let mut body_1 = self.body_1.borrow_mut();
        let mut body_2 = self.body_2.borrow_mut();
        let dv = body_2.velocity + body_2.angular_velocity.cross(self.r2)
            - body_1.velocity
            - body_1.angular_velocity.cross(self.r1);
        let impulse = self.m * (self.bias - dv - self.p * self.softness);
        body_1.velocity = body_1.velocity - impulse * body_1.inv_mass;
        body_1.angular_velocity -= body_1.inv_moi * self.r1.cross(impulse);

        body_2.velocity = body_2.velocity + impulse * body_2.inv_mass;
        body_2.angular_velocity += body_2.inv_moi * self.r2.cross(impulse);

        self.p = self.p + impulse;
    }
}

#[cfg(feature = "log")]
#[derive(Debug, Clone, serde::Serialize)]
pub struct JointLog {
    pub body1_id: usize,
    pub body2_id: usize,
    pub local_anchor_1: Vec2,
    pub local_anchor_2: Vec2,
    pub accumulated_impulse: Vec2,
    pub softness: f32,
    pub bias_factor: f32,
}

#[cfg(feature = "log")]
impl Joint {
    pub fn to_log(&self) -> JointLog {
        let body1 = self.body_1.borrow();
        let body2 = self.body_2.borrow();
        JointLog {
            body1_id: body1.id,
            body2_id: body2.id,
            local_anchor_1: self.local_anchor_1,
            local_anchor_2: self.local_anchor_2,
            accumulated_impulse: self.p,
            softness: self.softness,
            bias_factor: self.bias_factor,
        }
    }
}
