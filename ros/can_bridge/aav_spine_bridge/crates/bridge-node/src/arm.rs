//! arm_when_open, in Rust: arm the spine once the safety gate opens; the bench's and the
//! simulator's arm:=true. The same node as aav_spine's arm_when_open.py.
//!
//! Calls       spine/arm   std_srvs/SetBool (data: true), every `retry` s
//! Parameters  retry (1.0 s), give_up (120.0 s)
//!
//! spine_bridge refuses to arm while the output mux isn't OPEN (Ben, 24 Oct), so this asks
//! again each `retry` s while spine_bridge isn't up yet or the gate alone refuses, then
//! exits: 0 once armed, 1 on any other refusal (an e-stop: a person pressed it) or after
//! `give_up` s. It never arms a second time. Only the stand-ins start it (bench.launch.py,
//! ravensim.launch.py spine:=true); on the car a person arms. The rule is spine-bridge's
//! `try_again`.

use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{bail, Context as _, Result};
use rclrs::*;
use ros_env::std_srvs::srv::{SetBool, SetBool_Request, SetBool_Response};
use spine_bridge::{try_again, GATE_SHUT};

/// How often "waiting to arm" may be logged.
const SAY_EVERY: Duration = Duration::from_secs(10);

fn main() -> ExitCode {
    match run() {
        Ok(code) => ExitCode::from(code),
        Err(e) => {
            eprintln!("arm_when_open: {e:#}");
            ExitCode::from(1)
        }
    }
}

/// A read-only number (float or whole).
fn number(node: &Node, name: &str, default: f64) -> Result<f64> {
    let p = node
        .declare_parameter::<ParameterValue>(name)
        .default(ParameterValue::Double(default))
        .read_only()
        .with_context(|| format!("parameter {name} (a number)"))?;
    Ok(match p.get() {
        ParameterValue::Double(v) => v,
        ParameterValue::Integer(v) => v as f64,
        other => bail!("{name}:={other:?}: not a number"),
    })
}

/// Spin for up to `limit`, until `done` says so; false if the time ran out (or Ctrl-C).
fn spin_until(
    executor: &mut Executor,
    limit: Duration,
    stop: &AtomicBool,
    done: impl Fn() -> bool,
) -> bool {
    let end = Instant::now() + limit;
    while !done() {
        if Instant::now() >= end || stop.load(Ordering::SeqCst) {
            return false;
        }
        let _ = executor.spin(SpinOptions::default().timeout(Duration::from_millis(20)));
    }
    true
}

/// One /spine/arm call: (success, message). Not up, or no answer, counts as shut.
fn ask(
    executor: &mut Executor,
    client: &Client<SetBool>,
    timeout: Duration,
    stop: &AtomicBool,
) -> (bool, String) {
    if !spin_until(executor, timeout, stop, || {
        client.service_is_ready().unwrap_or(false)
    }) {
        return (
            false,
            format!("{GATE_SHUT}: spine_bridge has no /spine/arm yet"),
        );
    }
    let answer: Arc<Mutex<Option<SetBool_Response>>> = Arc::new(Mutex::new(None));
    let promise: Promise<SetBool_Response> = match client.call(&SetBool_Request { data: true }) {
        Ok(p) => p,
        Err(e) => return (false, format!("{GATE_SHUT}: /spine/arm: {e}")),
    };
    let slot = Arc::clone(&answer);
    let _waiting = executor.commands().run(async move {
        if let Ok(r) = promise.await {
            *slot
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner) = Some(r);
        }
    });
    let read = || {
        answer
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .clone()
    };
    spin_until(executor, timeout, stop, || read().is_some());
    match read() {
        Some(r) => (r.success, r.message),
        None => (false, format!("{GATE_SHUT}: no answer from /spine/arm")),
    }
}

fn run() -> Result<u8> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("arm_when_open")?;
    let retry = number(&node, "retry", 1.0)?;
    let give_up = number(&node, "give_up", 120.0)?;
    if !(retry > 0.0 && retry.is_finite()) {
        bail!("retry:={retry}: must be above 0 s");
    }
    let (retry, give_up) = (
        Duration::from_secs_f64(retry),
        Duration::from_secs_f64(give_up.max(0.0)),
    );
    let client = node.create_client::<SetBool>("spine/arm")?;
    let stop = Arc::new(AtomicBool::new(false));
    let s = Arc::clone(&stop);
    ctrlc::set_handler(move || s.store(true, Ordering::SeqCst))?;
    let logger = node.logger().clone();
    let start = Instant::now();
    while !stop.load(Ordering::SeqCst) {
        let asked = Instant::now();
        let (success, message) = ask(&mut executor, &client, retry, &stop);
        let waited = start.elapsed();
        if success {
            log_info!(&logger, "{message} after {:.0} s", waited.as_secs_f64());
            return Ok(0);
        }
        if stop.load(Ordering::SeqCst) {
            break;
        }
        if !try_again(success, &message, waited, give_up) {
            let gave_up = if waited >= give_up {
                format!(" (gave up after {:.0} s)", waited.as_secs_f64())
            } else {
                String::new()
            };
            log_error!(&logger, "not armed: {message}{gave_up}");
            return Ok(1);
        }
        log_info!(logger.throttle(SAY_EVERY), "waiting to arm: {message}");
        let rest = retry.saturating_sub(asked.elapsed());
        spin_until(&mut executor, rest, &stop, || false);
    }
    Ok(1)
}
