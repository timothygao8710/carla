use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender, TryRecvError};
use std::sync::Arc;
use std::time::Duration;

use crate::nav::{Field, Nav};
use crate::runtime::Budget;
use crate::Shared;

type Job = Box<dyn FnOnce() + Send>;

pub(crate) struct Pend {
    pub key: [f64; 4],
    pub t: f64,
    rx: Receiver<Field>,
    cancel: Arc<AtomicBool>,
}

#[derive(Default)]
pub(crate) struct FieldSvc {
    tx: Option<Sender<Job>>,
    dead: bool,
    pub zone: Option<Pend>,
    pub nav: Option<Pend>,
    pub sq: Option<Pend>,
    pub late: u64,
}

impl FieldSvc {
    fn sender(&mut self) -> Option<Sender<Job>> {
        if self.tx.is_none() && !self.dead {
            let (tx, rx) = mpsc::channel::<Job>();
            let h = std::thread::Builder::new().name("field".into()).stack_size(16 << 20).spawn(move || {
                while let Ok(j) = rx.recv() {
                    let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(j));
                }
            });
            match h {
                Ok(_) => self.tx = Some(tx),
                Err(_) => self.dead = true,
            }
        }
        self.tx.clone()
    }
}

pub(crate) fn poll(p: &mut Option<Pend>) -> Option<([f64; 4], f64, Field)> {
    let r = match p {
        Some(q) => q.rx.try_recv(),
        None => return None,
    };
    match r {
        Ok(f) => p.take().map(|q| (q.key, q.t, f)),
        Err(TryRecvError::Empty) => None,
        Err(TryRecvError::Disconnected) => {
            *p = None;
            None
        }
    }
}

fn near(a: &[f64; 4], b: &[f64; 4], tol: f64) -> bool {
    (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2]) <= tol && (a[3] - b[3]).abs() < 1e-9
}

/// The field when it is ready while the budget keeps keep_ms; otherwise it stays pending in p.
#[allow(clippy::too_many_arguments)]
pub(crate) fn request(
    svc: &mut FieldSvc,
    which: u8,
    shared: &Arc<Shared>,
    key: [f64; 4],
    tol: f64,
    t: f64,
    budget: &Budget,
    keep_ms: f64,
    build: impl FnOnce(&Nav) -> Field + Send + 'static,
) -> Option<Field> {
    if budget.is_unlimited() {
        return Some(build(&shared.nav));
    }
    let tx = svc.sender();
    let p = match which {
        0 => &mut svc.zone,
        1 => &mut svc.nav,
        _ => &mut svc.sq,
    };
    let same = p.as_ref().map_or(false, |q| near(&q.key, &key, tol));
    if !same {
        if let Some(q) = p.take() {
            q.cancel.store(true, Ordering::Relaxed);
        }
        let (rtx, rrx) = mpsc::channel();
        let cancel = Arc::new(AtomicBool::new(false));
        let c2 = cancel.clone();
        let sh = shared.clone();
        let job: Job = Box::new(move || {
            if c2.load(Ordering::Relaxed) {
                return;
            }
            let f = build(&sh.nav);
            let _ = rtx.send(f);
        });
        let sent = match tx {
            Some(tx) => tx.send(job).map_err(|e| e.0),
            None => Err(job),
        };
        if let Err(job) = sent {
            job();
        }
        *p = Some(Pend { key, t, rx: rrx, cancel });
    }
    let wait = budget.remaining_ms() - keep_ms;
    let r = match p {
        Some(q) => {
            if wait > 0.0 {
                q.rx.recv_timeout(Duration::from_secs_f64(wait / 1000.0))
            } else {
                q.rx.try_recv().map_err(|e| match e {
                    TryRecvError::Empty => RecvTimeoutError::Timeout,
                    TryRecvError::Disconnected => RecvTimeoutError::Disconnected,
                })
            }
        }
        None => return None,
    };
    match r {
        Ok(f) => {
            *p = None;
            Some(f)
        }
        Err(RecvTimeoutError::Timeout) => {
            svc.late += 1;
            None
        }
        Err(RecvTimeoutError::Disconnected) => {
            *p = None;
            None
        }
    }
}
