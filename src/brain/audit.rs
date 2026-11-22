use super::math::*;
use super::{Brain, FireSnap, Tgt};
use crate::proto::{Action, ActorKind, Obs, WeaponKind};
use crate::world::World;
use std::collections::BTreeMap;

#[derive(Default)]
pub(crate) struct Audit {
    pub on: bool,
    pub gate: &'static str,
    pub c: BTreeMap<String, f64>,
}

impl Audit {
    pub fn new() -> Self {
        Audit { on: std::env::var("MECH_AUDIT").map_or(false, |v| v == "1"), gate: "", c: BTreeMap::new() }
    }
    fn add(&mut self, k: &str, v: f64) {
        *self.c.entry(k.to_string()).or_insert(0.0) += v;
    }
}

impl Drop for Audit {
    fn drop(&mut self) {
        if self.on && !self.c.is_empty() {
            let s: Vec<String> = self.c.iter().map(|(k, v)| format!("\"{}\":{}", k, v)).collect();
            eprintln!("AUDIT {{{}}}", s.join(","));
        }
    }
}

impl Brain {
    pub(crate) fn audit_step(&mut self, obs: &Obs, world: &World, s: &[FireSnap; 5], act: &Action) {
        if !self.au.on {
            return;
        }
        let t = obs.t;
        let ms = self.t_act.elapsed().as_secs_f64() * 1000.0;
        let mx = self.au.c.get("act_ms_max").copied().unwrap_or(0.0);
        if ms > mx {
            self.au.c.insert("act_ms_max".to_string(), ms);
        }
        let me = &obs.me;
        let muzzle = up(me.pos, me.stats.muzzle_height);
        for e in &obs.events {
            if let crate::proto::Event::Damaged(d) = e {
                let k = if d.rear { "taken_rear" } else { "taken_front" };
                self.au.add(k, d.amount);
                let vis = d.by.map_or(false, |b| obs.mechs.iter().any(|m| m.id == b));
                let tgt = s[0].tfeet.is_some();
                let off = d.bearing.map_or(-1.0, |b| adiff(b, me.aim[0]).abs());
                let ctx = format!("{}_{}_{}_{}", if d.rear { "R" } else { "F" }, if vis { "vis" } else { "unseen" }, if tgt { "tgt" } else { "notgt" }, format!("{:?}", self.mode));
                self.au.add(&format!("ctx_{ctx}"), d.amount);
                if d.rear && off >= 0.0 {
                    self.au.add(&format!("rearoff_{}", ((off.to_degrees() / 30.0) as i64) * 30), d.amount);
                }
                self.au.add(&format!("taken_w_{}", d.weapon), d.amount);
            }
            if let crate::proto::Event::Kill(_) = e {
                self.au.add(&format!("kill_t{:03}", ((t / 15.0) as i64) * 15), 1.0);
            }
            if let crate::proto::Event::Hit(h) = e {
                self.au.add(&format!("dealt_w_{}", h.weapon), h.amount);
            }
        }
        let fc_tgt = s[0].tfeet;
        let cand = self.mem.tracks.iter().filter(|tr| tr.kind == ActorKind::Mech && t - tr.last_seen <= 0.15).find(|tr| {
            self.target == Tgt::Mech(tr.id) || fc_tgt.map_or(false, |f| dist(f, tr.pos) < 0.5)
        });
        let Some(tr) = cand.cloned() else { return };
        let at = self.aim_target(&tr);
        let c = up(at.feet, at.height * 0.5);
        let d = dist(c, muzzle);
        let clear = world.ray(muzzle, c).is_none();
        let harmless = self.gear_harmless(&tr, t, d);
        let fire = act.fire.unwrap_or([false, false]);
        let fc_is = fc_tgt.map_or(false, |f| dist(f, tr.pos) < 0.5);
        for cls in ["all", "nh"] {
            if cls == "nh" && harmless {
                continue;
            }
            for (i, w) in me.weapons.iter().enumerate() {
                let Some(w) = w else { continue };
                match w.kind {
                    WeaponKind::Gun => {
                        if !clear || d > w.stats.range {
                            continue;
                        }
                        self.au.add(&format!("{cls}_gun_n"), 1.0);
                        let st = if w.reload > 0.0 {
                            "reloading".to_string()
                        } else if w.ammo <= 0.0 {
                            "empty".to_string()
                        } else if fire[i] {
                            "firing".to_string()
                        } else {
                            let why = if !fc_is {
                                "target_other"
                            } else if !s[0].fire[i] {
                                if self.au.gate.is_empty() { "unknown" } else { self.au.gate }
                            } else if !s[2].fire[i] {
                                "laser_post"
                            } else if !s[3].fire[i] {
                                "safety_es"
                            } else {
                                "halt_or_other"
                            };
                            format!("blocked_{why}")
                        };
                        self.au.add(&format!("{cls}_gun_{st}"), 1.0);
                        if s[1].aim != s[0].aim {
                            self.au.add(&format!("{cls}_gun_aim_sweep"), 1.0);
                        }
                        if s[2].aim != s[1].aim {
                            self.au.add(&format!("{cls}_gun_aim_laserpost"), 1.0);
                        }
                    }
                    WeaponKind::Laser => {
                        if !clear || d > w.stats.range {
                            continue;
                        }
                        self.au.add(&format!("{cls}_laser_n"), 1.0);
                        let full = w.stats.charge.max(0.05);
                        let st = if w.charge > 0.0 {
                            if w.charge >= full - 0.01 { if fire[i] { "full_holding" } else { "full_release" } } else { "charging" }
                        } else if w.cooldown > 0.0 {
                            "cooldown"
                        } else if fire[i] {
                            "start"
                        } else if !fc_is {
                            "idle_target_other"
                        } else {
                            "idle"
                        };
                        self.au.add(&format!("{cls}_laser_{st}"), 1.0);
                    }
                    _ => {}
                }
            }
        }
    }
}
