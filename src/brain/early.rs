use super::math::*;
use super::aim::FireOut;
use super::memory::Track;
use super::Brain;
use crate::proto::{ActorKind, Obs, WeaponKind};

impl Brain {
    fn es_laser(obs: &Obs) -> bool {
        obs.me.weapons.iter().flatten().any(|w| w.kind == WeaponKind::Laser)
    }

    pub(crate) fn es_hold(&self, obs: &Obs, tr: &Track) -> bool {
        let p = &self.p;
        if p.es_on < 0.5 || tr.kind != ActorKind::Mech {
            return false;
        }
        let t = obs.t;
        if t - tr.last_dmg_t < p.es_att_s {
            return false;
        }
        let me = &obs.me;
        let hpf = frac(me.hp, me.max_hp);
        let early = t < p.es_until && !Self::es_laser(obs);
        let weak = hpf < p.es_hp;
        if !(early || weak) {
            return false;
        }
        let d = dist(tr.pos_at(t), me.pos);
        if d < p.es_free_d && !tr.has(WeaponKind::Laser) && tr.cubes <= p.es_free_cubes && !weak {
            return false;
        }
        true
    }

    pub(crate) fn es_filter(&self, obs: &Obs, fc: &mut FireOut) {
        if self.p.es_on < 0.5 || self.p.es_cone <= 0.0 {
            return;
        }
        let me = &obs.me;
        let yaw = fc.aim[0];
        if !yaw.is_finite() {
            return;
        }
        let mut best: Option<(f64, i64)> = None;
        for a in &obs.mechs {
            if a.kind != ActorKind::Mech || a.id == me.id || !a.pos.iter().all(|x| x.is_finite()) {
                continue;
            }
            let dx = a.pos[0] - me.pos[0];
            let dy = a.pos[1] - me.pos[1];
            let d = dx.hypot(dy);
            if d < 1.0 || d > 400.0 {
                continue;
            }
            let e = adiff(yaw, dy.atan2(dx)).abs();
            let cone = self.p.es_cone.max((6.0 / d).atan());
            if e < cone && best.map_or(true, |b| d < b.0) {
                best = Some((d, a.id));
            }
        }
        let Some((_, id)) = best else { return };
        let Some(tr) = self.mem.tracks.iter().find(|x| x.id == id) else { return };
        if !self.es_hold(obs, tr) {
            return;
        }
        for (i, w) in me.weapons.iter().enumerate().take(2) {
            if let Some(w) = w {
                if w.kind == WeaponKind::Laser && w.charge > 0.0 {
                    continue;
                }
                fc.fire[i] = false;
            }
        }
    }
}
