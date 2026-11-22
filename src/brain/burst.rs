use super::math::*;
use super::plan::MoveOut;
use super::aim::FireOut;
use super::{Brain, Tgt};
use crate::nav::CELL;
use crate::proto::{ActorKind, Obs, WeaponKind};
use crate::world::World;

#[derive(Clone, Debug, Default)]
pub(crate) struct BurstSt {
    pub dodge: Option<(V3, f64, i64)>,
    pub side: f64,
    pub side_t: f64,
    pub hidden_t: f64,
}

struct Thr {
    id: i64,
    eye: V3,
    tp: V3,
    held: f64,
}

impl Brain {
    fn bu_threats(&self, obs: &Obs) -> Vec<Thr> {
        let t = obs.t;
        let pos = obs.me.pos;
        let mut v = Vec::new();
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech || !tr.charging || t - tr.last_seen > 0.25 || !tr.has(WeaponKind::Laser) {
                continue;
            }
            let tp = tr.pos_at(t);
            let d = dist(tp, pos);
            if d < 12.0 || d > self.cfg.weapons.laser.range + 20.0 {
                continue;
            }
            let brg = (pos[1] - tp[1]).atan2(pos[0] - tp[0]);
            if adiff(tr.aim[0], brg).abs() > self.p.bu_cone + (5.0f64).atan2(d) {
                continue;
            }
            let s = 1.0 + self.cfg.cubes.size_per_cube * tr.cubes.max(0.0);
            v.push(Thr { id: tr.id, eye: up(tp, self.cfg.mech.eye_height * s), tp, held: t - tr.charge_since });
        }
        v
    }

    fn bu_seen(world: &World, eyes: &[V3], q: V3, h: f64) -> usize {
        eyes.iter().filter(|e| world.ray(**e, up(q, h * 0.5)).is_none() || world.ray(**e, up(q, h * 0.95)).is_none()).count()
    }

    pub(crate) fn burst_post(&mut self, obs: &Obs, world: &World, fc: &mut FireOut, mv: &mut MoveOut) {
        let p = self.p.clone();
        let t = obs.t;
        let me = &obs.me;
        let pos = me.pos;
        if p.bu_regen_hp > 0.0 {
            let hpf = frac(me.hp, me.max_hp);
            let charging = me.weapons.iter().flatten().any(|w| w.charge > 0.0);
            let mech_tgt = matches!(self.target, Tgt::Mech(_));
            let recent = self.mem.tracks.iter().any(|tr| tr.kind == ActorKind::Mech && (t - tr.last_seen < p.bu_regen_seen_s || t - tr.last_dmg_t < p.bu_regen_seen_s));
            if hpf < p.bu_regen_hp && !mech_tgt && !recent && !charging {
                fc.fire = [false, false];
            }
        }
        if p.bu_break <= 0.0 {
            return;
        }
        let h = self.cfg.mech.height;
        let thr = self.bu_threats(obs);
        if let Some((q, until, id)) = self.bu.dodge {
            let still = thr.iter().any(|x| x.id == id);
            if t > until || !still {
                self.bu.dodge = None;
            } else {
                let dx = q[0] - pos[0];
                let dy = q[1] - pos[1];
                let l = dx.hypot(dy);
                if l > 0.7 {
                    mv.dir = [dx / l, dy / l];
                    mv.boost = me.energy > 6.0;
                    mv.jump = false;
                }
                return;
            }
        }
        if thr.len() < p.bu_min_thr as usize || thr.is_empty() {
            return;
        }
        let eyes: Vec<V3> = thr.iter().map(|x| x.eye).collect();
        let seen_now = Self::bu_seen(world, &eyes, pos, h);
        if seen_now == 0 {
            return;
        }
        let main = thr.iter().max_by(|a, b| a.held.total_cmp(&b.held)).map(|x| (x.id, x.tp)).unwrap();
        let shared = self.shared.clone();
        let nav = &shared.nav;
        let speed = me.stats.boost_speed.max(me.stats.walk_speed).max(1.0);
        let r = (p.bu_reach_s * speed).min(p.bu_r).max(2.0);
        let (cnx, cny, o) = nav.grid();
        let i0 = (((pos[0] - r - o[0]) / CELL).floor().max(0.0)) as usize;
        let j0 = (((pos[1] - r - o[1]) / CELL).floor().max(0.0)) as usize;
        let i1 = ((((pos[0] + r - o[0]) / CELL).ceil()).max(0.0) as usize).min(cnx.saturating_sub(1));
        let j1 = ((((pos[1] + r - o[1]) / CELL).ceil()).max(0.0) as usize).min(cny.saturating_sub(1));
        let mut best: Option<(f64, V3)> = None;
        'outer: for cj in j0..=j1 {
            for ci in i0..=i1 {
                for nd in nav.cell_node_ids(ci, cj) {
                    let q = nav.node_pos(nd);
                    let d = dist(q, pos);
                    if d > r || (q[2] - pos[2]).abs() > 2.5 || best.map_or(false, |b| d >= b.0) {
                        continue;
                    }
                    if Self::bu_seen(world, &eyes, q, h) == 0 && nav.clear(pos, q) {
                        best = Some((d, q));
                    }
                }
                if self.spent_ms() > p.bu_ms {
                    break 'outer;
                }
            }
        }
        if let Some((_, q)) = best {
            self.bu.dodge = Some((q, t + p.bu_hold_s, main.0));
            let dx = q[0] - pos[0];
            let dy = q[1] - pos[1];
            let l = dx.hypot(dy).max(1e-6);
            mv.dir = [dx / l, dy / l];
            mv.boost = me.energy > 6.0;
            mv.jump = false;
            return;
        }
        if p.bu_dodge > 0.0 && me.energy > p.bu_dodge_energy {
            if t > self.bu.side_t {
                self.bu.side = if self.bu.side > 0.0 { -1.0 } else { 1.0 };
                self.bu.side_t = t + self.rng.range(0.35, 0.9);
            }
            let tp = main.1;
            let bx = pos[0] - tp[0];
            let by = pos[1] - tp[1];
            let l = bx.hypot(by).max(1e-6);
            let perp = [-by / l * self.bu.side, bx / l * self.bu.side];
            let probe = [pos[0] + perp[0] * 6.0, pos[1] + perp[1] * 6.0, pos[2]];
            let ok = nav.snap(probe).map_or(false, |s| dist(s, probe) < 3.0 && nav.clear(pos, s));
            let dir = if ok { perp } else { [-perp[0], -perp[1]] };
            if !ok {
                self.bu.side = -self.bu.side;
            }
            mv.dir = dir;
            mv.boost = true;
        }
        let _ = fc;
    }
}
