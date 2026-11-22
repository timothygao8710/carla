use super::aim::FireOut;
use super::math::*;
use super::{Brain, Mode, Tgt};
use crate::proto::{ActorKind, Obs, WeaponKind};

impl Brain {
    pub(crate) fn hold_lost_charge(&self, obs: &Obs, out: &mut FireOut) {
        let mem_s = self.p.lz_hold_lost;
        if mem_s <= 0.0 || matches!(self.mode, Mode::Retreat | Mode::Zone) {
            return;
        }
        let me = &obs.me;
        let t = obs.t;
        let Some((i, w)) = me.weapons.iter().enumerate().find_map(|(i, w)| w.as_ref().filter(|w| w.kind == WeaponKind::Laser).map(|w| (i, w))) else { return };
        if !(w.charge > 0.0) || i > 1 {
            return;
        }
        let full = w.stats.charge.max(0.05);
        if w.charge >= full + (w.stats.hold_s - 0.3).max(0.0) {
            return;
        }
        let tid = match self.target {
            Tgt::Mech(id) => id,
            _ => -1,
        };
        let mut best: Option<(f64, V3)> = None;
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech || t - tr.last_seen > mem_s {
                continue;
            }
            let tp = tr.pos_at(t.min(tr.last_seen + 1.0));
            let d = dist(tp, me.pos);
            if !(d <= w.stats.range) {
                continue;
            }
            let s = (t - tr.last_seen) * 100.0 + d * 0.1 - if tr.id == tid { 50.0 } else { 0.0 };
            if best.map_or(true, |b| s < b.0) {
                best = Some((s, tp));
            }
        }
        let Some((_, tp)) = best else { return };        out.fire[i] = true;
        out.charging = true;
        out.armed = true;
        let muzzle = up(me.pos, me.stats.muzzle_height);
        let a = dir_to_aim(sub(up(tp, self.cfg.mech.height * 0.5), muzzle));
        if a[0].is_finite() && a[1].is_finite() {
            out.aim = a;
        }
    }
}
