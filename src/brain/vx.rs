use super::math::*;
use super::memory::Track;
use super::strategy::ZoneInfo;
use super::{Brain, Goal};
use crate::nav::Nav;
use crate::proto::{ActorKind, Event, Obs, WeaponKind};
use crate::world::World;

#[derive(Default, Clone, Debug)]
pub(crate) struct Vx {
    pub last_line_t: f64,
    pub n_line: usize,
    pub n_strong: usize,
    pub strong_kill: bool,
    pub regroup: bool,
    pub hold: bool,
    pub kite: bool,
    pub kite_low_t: f64,
    pub kite_goal: Option<(V3, f64)>,
    pub eyes: Vec<V3>,
    pub seeking: bool,
    pub sk_ban: Vec<(i64, f64)>,
}

impl Brain {
    pub(crate) fn vx_strong(&self, tr: &Track) -> bool {
        tr.has(WeaponKind::Laser) || self.sf_not_harmless(tr) || self.order.dmg(tr.id) >= self.p.gear_harmless_dmg
    }

    pub(crate) fn tk_bonus(&self, tr: &Track) -> f64 {
        if self.p.tk_on < 0.5 || tr.kind != ActorKind::Mech || !self.vx_strong(tr) {
            return 0.0;
        }
        self.p.tk_focus * (1.0 - frac(tr.hp, tr.max_hp))
    }

    pub(crate) fn tk_no_detour(&self) -> bool {
        self.p.tk_on >= 0.5 && self.wave.kills as f64 >= self.p.tk_kills
    }

    pub(crate) fn spare_tr(&self, tr: &Track, t: f64, pos: V3) -> bool {
        if !self.spare(tr.hp, tr.max_hp) {
            return false;
        }
        if self.p.ready_spare_harmless < 0.5 {
            return true;
        }
        t - tr.last_dmg_t > 5.0 || self.gear_harmless(tr, t, dist(tr.pos_at(t), pos))
    }

    pub(crate) fn vx_update(&mut self, obs: &Obs, world: &World) {
        let p = self.p.clone();
        if p.rg_on < 0.5 && p.kt_on < 0.5 {
            return;
        }
        let t = obs.t;
        let me = &obs.me;
        if self.vx.last_line_t == 0.0 {
            self.vx.last_line_t = -1e9;
        }
        for ev in &obs.events {
            if let Event::Kill(k) = ev {
                if let Some(tr) = self.mem.tracks.iter().find(|tr| tr.kind == ActorKind::Mech && tr.id == k.target) {
                    if self.vx_strong(tr) {
                        self.vx.strong_kill = true;
                    }
                }
            }
        }
        let body = up(me.pos, me.stats.height.max(1.0) * 0.6);
        let eye_h = self.cfg.mech.eye_height;
        let mut n = 0;
        let mut ns = 0;
        self.vx.eyes.clear();
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech {
                continue;
            }
            let hit = t - tr.last_dmg_t < 0.5;
            let fresh = t - tr.last_seen < 0.3;
            if !hit && !fresh {
                continue;
            }
            let e = up(tr.pos_at(t), eye_h);
            if hit || (dist(e, body) < 320.0 && world.sight(e, body)) {
                n += 1;
                if self.vx_strong(tr) {
                    ns += 1;
                    if self.vx.eyes.len() < 6 {
                        self.vx.eyes.push(e);
                    }
                }
            }
        }
        self.vx.n_line = n;
        self.vx.n_strong = ns;
        if n > 0 || t - self.mem.last_damage_t < 0.3 {
            self.vx.last_line_t = t;
        }
        let hpf = frac(me.hp, me.max_hp);
        if p.rg_on >= 0.5 {
            let quiet = t - self.vx.last_line_t >= p.rg_quiet_s && t - self.mem.last_damage_t >= p.rg_quiet_s;
            let low = hpf < p.rg_hp || (self.vx.strong_kill && p.rg_strong >= 0.5 && hpf < p.rg_exit);
            if hpf >= p.rg_exit {
                self.vx.strong_kill = false;
            }
            self.vx.regroup = if self.vx.regroup { quiet && hpf < p.rg_exit } else { quiet && low && me.in_zone };
            self.vx.hold = self.vx.regroup;
        } else {
            self.vx.strong_kill = false;
        }
        if p.kt_on >= 0.5 {
            let kills = self.wave.kills as f64;
            if kills < p.kt_kills {
                self.vx.kite = false;
            } else if ns >= p.kt_n.max(1.0) as usize {
                self.vx.kite = true;
                self.vx.kite_low_t = t;
            } else if self.vx.kite && ns >= 2 {
                self.vx.kite_low_t = t;
            } else if self.vx.kite && t - self.vx.kite_low_t > p.kt_exit_s {
                self.vx.kite = false;
            }
        }
    }

    pub(crate) fn kite_goal(&mut self, obs: &Obs, world: &World, nav: &Nav, zi: &ZoneInfo) -> Goal {
        let t = obs.t;
        let pos = obs.me.pos;
        if let Some((q, tq)) = self.vx.kite_goal {
            if t - tq < 0.5 && dist(q, pos) > 6.0 {
                return Goal::Nav { pos: q, arrive: 2.0 };
            }
        }
        if self.vx.eyes.is_empty() {
            self.vx.kite_goal = None;
            return Goal::None;
        }
        let k = self.vx.eyes.len() as f64;
        let cx = self.vx.eyes.iter().map(|e| e[0]).sum::<f64>() / k;
        let cy = self.vx.eyes.iter().map(|e| e[1]).sum::<f64>() / k;
        let away = unit2(pos[0] - cx, pos[1] - cy);
        let tc = unit2(zi.safe[0] - pos[0], zi.safe[1] - pos[1]);
        let base = away[1].atan2(away[0]);
        let h = obs.me.stats.height.max(1.0);
        let r = self.p.kt_step.max(10.0);
        let mut best: Option<(f64, V3)> = None;
        for da in [0.0f64, 0.5, -0.5, 1.0, -1.0] {
            if self.spent_ms() > 9.0 {
                break;
            }
            let a = base + da;
            let x = pos[0] + r * a.cos();
            let y = pos[1] + r * a.sin();
            let q0 = [x, y, fin(world.floor_below(x, y, pos[2] + 3.0), pos[2])];
            let Some(q) = nav.snap(q0) else { continue };
            if dist(q, q0) > 8.0 || !q.iter().all(|v| v.is_finite()) || dist2v(q, zi.cur_c) > zi.cur_r - 12.0 {
                continue;
            }
            let seen = self.vx.eyes.iter().filter(|e| world.sight(**e, up(q, h * 0.6))).count() as f64;
            let dir = unit2(q[0] - pos[0], q[1] - pos[1]);
            let s = seen * 2.0 + da.abs() * 0.5 - 0.6 * (dir[0] * tc[0] + dir[1] * tc[1]);
            if best.map_or(true, |b| s < b.0) {
                best = Some((s, q));
            }
        }
        match best {
            Some((_, q)) => {
                self.vx.kite_goal = Some((q, t));
                Goal::Nav { pos: q, arrive: 2.0 }
            }
            None => {
                self.vx.kite_goal = None;
                Goal::None
            }
        }
    }

    pub(crate) fn dp_pick(&self, obs: &Obs, drop: Option<(V3, f64, f64)>, tgt: Option<&Track>, want_retreat: bool, hpf: f64) -> Option<(V3, f64, f64)> {
        let p = &self.p;
        if p.dp_on < 0.5 {
            return None;
        }
        let d = drop?;
        if d.2 > p.dp_dist {
            return None;
        }
        let strong_fight = tgt.map_or(false, |tr| self.vx_strong(tr) && obs.t - tr.last_seen < 1.0);
        let leave = hpf < p.dp_leave_hp && d.2 < p.dp_leave_r;
        if leave || (!want_retreat && !strong_fight) {
            Some(d)
        } else {
            None
        }
    }
}

impl Brain {
    pub(crate) fn seek_goal(&mut self, obs: &Obs, zi: &ZoneInfo) -> Option<(i64, V3)> {
        let p = self.p.clone();
        self.vx.seeking = false;
        if p.sk_on < 0.5 {
            return None;
        }
        let me = &obs.me;
        let t = obs.t;
        if !me.weapons.iter().flatten().any(|w| w.kind == WeaponKind::Laser) || frac(me.hp, me.max_hp) < p.sk_min_hp {
            return None;
        }
        self.vx.sk_ban.retain(|b| b.1 > t);
        let pos = me.pos;
        let mut best: Option<(f64, i64, V3)> = None;
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech || t - tr.last_seen > p.sk_mem_s || self.vx.sk_ban.iter().any(|b| b.0 == tr.id) {
                continue;
            }
            let q = tr.pos_at(t);
            let d = dist(q, pos);
            if d > p.sk_dist || !self.in_play_pub(obs, zi, q, d.min(200.0)) {
                continue;
            }
            if d < 20.0 && t - tr.last_seen > 2.0 {
                continue;
            }
            let iso = !self.mem.tracks.iter().any(|o| o.kind == ActorKind::Mech && o.id != tr.id && t - o.last_seen < 5.0 && dist(o.pos_at(t), q) < 60.0);
            let mut s = d + 80.0 * frac(tr.hp, tr.max_hp) + (t - tr.last_seen) * 3.0 - if iso { 30.0 } else { 0.0 };
            if self.goal_id == tr.id {
                s -= 25.0;
            }
            if best.map_or(true, |b| s < b.0) {
                best = Some((s, tr.id, q));
            }
        }
        if best.is_none() {
            let sc = &self.cfg.sound;
            for h in &self.mem.sounds {
                if h.kind == "npc" || t - h.t > 4.0 {
                    continue;
                }
                let r = match h.dist {
                    crate::proto::SoundDist::Near => sc.near * 0.7,
                    crate::proto::SoundDist::Mid => (sc.near + sc.mid) * 0.5,
                    _ => sc.mid * 1.3,
                };
                let r = r.max(20.0).min(p.sk_dist);
                let q = [pos[0] + r * h.bearing.cos(), pos[1] + r * h.bearing.sin(), pos[2]];
                if !q.iter().all(|v| v.is_finite()) || !self.in_play_pub(obs, zi, q, r) {
                    continue;
                }
                let s = r + (t - h.t) * 10.0 + 100.0;
                if best.map_or(true, |b| s < b.0) {
                    best = Some((s, -2, q));
                }
            }
        }
        if let Some((_, id, q)) = best {
            if id >= 0 && dist(q, pos) < 20.0 {
                self.vx.sk_ban.push((id, t + 8.0));
            }
            self.vx.seeking = true;
            return Some((id, q));
        }
        None
    }
}
