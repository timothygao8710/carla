use std::collections::VecDeque;

use super::aim::FireOut;
use super::math::*;
use super::Brain;
use crate::proto::{ActorKind, Obs, ProjKind, WeaponKind};
use crate::world::World;

pub(crate) struct Dg {
    pub side: f64,
    pub keep: Option<(V2, f64, i64, f64)>,
    pub next: f64,
    pub last_flip: f64,
    pub boost_on: bool,
    pub active_t: f64,
    pub lz_key: (i64, f64),
    pub lz_until: f64,
    pub cnt: u32,
    pub cnt_id: i64,
    known: VecDeque<i64>,
}

impl Dg {
    pub(crate) fn new() -> Dg {
        Dg { side: 1.0, keep: None, next: 0.0, last_flip: -1e9, boost_on: true, active_t: -1e9, lz_key: (-1, -1e9), lz_until: -1e9, cnt: 0, cnt_id: -2, known: VecDeque::new() }
    }
}

impl Brain {
    pub(crate) fn dg_flip(&mut self, t: f64, boost: bool, ft: f64) {
        let p = self.p.clone();
        self.dg.side = -self.dg.side;
        let per = if boost { self.rng.range(p.dg_pb_lo, p.dg_pb_hi.max(p.dg_pb_lo)) } else { self.rng.range(p.dg_pw_lo, p.dg_pw_hi.max(p.dg_pw_lo)) };
        self.dg.next = t + per.max(p.dg_ftk * ft).max(0.15);
        self.dg.last_flip = t;
        self.dg.cnt = 0;
    }

    pub(crate) fn dg_observe(&mut self, obs: &Obs) {
        let t = obs.t;
        let me = obs.me.pos;
        let mc = [me[0], me[1], me[2] + obs.me.stats.height * 0.5];
        for pr in &obs.projectiles {
            if pr.kind != ProjKind::Gun || self.dg.known.contains(&pr.id) {
                continue;
            }
            if !pr.pos.iter().chain(pr.vel.iter()).all(|x| x.is_finite()) {
                continue;
            }
            self.dg.known.push_back(pr.id);
            if self.dg.known.len() > 160 {
                self.dg.known.pop_front();
            }
            let sp = len(pr.vel);
            if sp < 50.0 {
                continue;
            }
            let u = scale(pr.vel, 1.0 / sp);
            let rel = sub(mc, pr.pos);
            let along = dot(rel, u);
            if along < 0.0 || len(sub(rel, scale(u, along))) > 12.0 {
                continue;
            }
            let mut best: Option<(f64, i64)> = None;
            for m in &obs.mechs {
                if m.kind != ActorKind::Mech {
                    continue;
                }
                let w = sub([m.pos[0], m.pos[1], m.pos[2] + obs.me.stats.muzzle_height], pr.pos);
                let s = -dot(w, u);
                if s < 0.0 || s > 400.0 {
                    continue;
                }
                let perp = len(add(w, scale(u, s)));
                if perp < 4.0 && best.map_or(true, |b| perp < b.0) {
                    best = Some((perp, m.id));
                }
            }
            if let Some((_, id)) = best {
                if id == self.dg.cnt_id {
                    self.dg.cnt += 1;
                }
            }
        }
        if self.dg.cnt_id == -1 {
            if let Some(u) = &self.mem.unseen {
                for s in &obs.sounds {
                    if s.kind == "gun" && s.age < 0.12 && adiff(s.bearing, u.bearing).abs() < 0.4 {
                        self.dg.cnt += 1;
                        break;
                    }
                }
            }
        }
        let _ = t;
    }

    pub(crate) fn dg_face(&mut self, obs: &Obs, world: &World, fc: &mut FireOut) {
        if fc.target.is_some() || fc.charging || fc.fire[0] || fc.fire[1] {
            return;
        }
        let p = self.p.clone();
        let t = obs.t;
        let me = &obs.me;
        let c = up(me.pos, me.stats.height * 0.5);
        let mut best: Option<(f64, f64)> = None;
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech || t - tr.last_seen > 2.0 || !(tr.has(WeaponKind::Gun) || tr.weapons.is_empty()) {
                continue;
            }
            let tp = tr.pos_at(t);
            let d = dist2(tp, me.pos);
            if d > self.cfg.weapons.gun.range + 10.0 || best.map_or(false, |b| d >= b.0) {
                continue;
            }
            let from = (c[1] - tp[1]).atan2(c[0] - tp[0]);
            if adiff(tr.aim[0], from).abs() > 0.35 {
                continue;
            }
            let s = 1.0 + self.cfg.cubes.size_per_cube * tr.cubes.max(0.0);
            if !world.sight(up(tp, self.cfg.mech.eye_height * s), c) {
                continue;
            }
            best = Some((d, (tp[1] - me.pos[1]).atan2(tp[0] - me.pos[0])));
        }
        if best.is_none() {
            if let Some(u) = &self.mem.unseen {
                if t - u.t < 1.5 {
                    best = Some((1e9, u.bearing));
                }
            }
        }
        if let Some((_, b)) = best {
            if adiff(me.aim[0], b).abs() > p.dg_rear_deg.to_radians() {
                fc.aim = [b, 0.0];
            }
        }
    }
}
