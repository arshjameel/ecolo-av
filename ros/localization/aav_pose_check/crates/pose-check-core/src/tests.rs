//! The checker on made-up streams: NDT at 10 Hz, the GPS at 5 Hz, the car driving east
//! at 1 m/s.

use super::*;

const NDT_COV: Cov2 = Cov2 {
    xx: 0.0004,
    xy: 0.0,
    yy: 0.0004,
};
const GPS_COV: Cov2 = Cov2 {
    xx: 0.25,
    xy: 0.0,
    yy: 0.25,
};

/// Where the car truly is at t.
fn truth(t: f64) -> Pose2 {
    Pose2::new(t, 0.0, 0.0)
}

/// Drives from `from` to `to` s: NDT gives `ndt(t)` (None: no fit, LOST) with `score`,
/// the GPS `gps(t)` (None: silent). Returns every tick (one per NDT cloud) and the
/// counts of poses passed to the EKF: (NDT, GPS).
struct Run {
    ticks: Vec<Tick>,
    ndt_passed: usize,
    gps_passed: usize,
}

fn drive(
    c: &mut Checker,
    from: f64,
    to: f64,
    ndt: impl Fn(f64) -> Option<(Pose2, f64)>,
    gps: impl Fn(f64) -> Option<(f64, f64)>,
) -> Run {
    let mut run = Run {
        ticks: Vec::new(),
        ndt_passed: 0,
        gps_passed: 0,
    };
    let steps = ((to - from) * 10.0).round() as usize;
    for k in 0..steps {
        let t = from + k as f64 * 0.1;
        match ndt(t) {
            Some((pose, score)) => {
                c.ndt_status(t, NdtState::Tracking, Some(score));
                run.ndt_passed += usize::from(c.ndt_pose(t, pose, NDT_COV));
            }
            None => c.ndt_status(t, NdtState::Lost, None),
        }
        c.heading(0.0);
        if k % 2 == 0 {
            if let Some((x, y)) = gps(t) {
                c.gps_fix(true);
                run.gps_passed += usize::from(c.gps(t, x, y, GPS_COV));
            }
        }
        run.ticks.push(c.tick(t));
    }
    run
}

fn checker() -> Checker {
    Checker::new(Settings::default()).expect("the defaults are good")
}

fn good_ndt(t: f64) -> Option<(Pose2, f64)> {
    Some((truth(t), 3.0))
}

/// The GPS with 0.3 m of made-up noise that sums to nothing over a few fixes.
fn clean_gps(t: f64) -> Option<(f64, f64)> {
    let k = (t * 5.0).round() as i64;
    let n = [0.3, -0.2, 0.1, -0.3, 0.1][k.rem_euclid(5) as usize];
    Some((t + n, -n))
}

#[test]
fn ndt_leads_and_the_gps_only_checks_while_they_agree() {
    let mut c = checker();
    let run = drive(&mut c, 0.0, 30.0, good_ndt, clean_gps);
    let last = &run.ticks.last().unwrap().report;
    assert_eq!(last.source, Source::Ndt);
    assert_eq!(last.level, Level::Ok);
    assert_eq!(c.verdict(), Verdict::Agree);
    assert_eq!(run.ndt_passed, 300);
    assert_eq!(
        run.gps_passed, 0,
        "the GPS never reaches the EKF while NDT leads"
    );
    assert!(last.gap.unwrap() < 0.5);
}

#[test]
fn a_gps_that_drifts_is_named_and_never_moves_the_pose() {
    let mut c = checker();
    // 5 m north over 60 s: never a jump, never silent.
    let drift = |t: f64| clean_gps(t).map(|(x, y)| (x, y + 5.0 * (t / 60.0).min(1.0)));
    let run = drive(&mut c, 0.0, 90.0, good_ndt, drift);
    assert_eq!(run.gps_passed, 0);
    assert!(run.ticks.iter().all(|t| t.report.source == Source::Ndt));
    let named = run
        .ticks
        .iter()
        .position(|t| matches!(t.report.gps, Some(GpsDoubt::Disagrees { .. })))
        .expect("the drift is named");
    // Named once the window's mean gap passes disagree_m (1.5 m: about 20-25 s in).
    let at = named as f64 * 0.1;
    assert!((15.0..35.0).contains(&at), "named at {at} s");
    assert_eq!(run.ticks.last().unwrap().report.level, Level::Warn);
}

#[test]
fn a_drifted_gps_is_not_the_fall_back_when_ndt_is_lost() {
    let mut c = checker();
    let drift = |t: f64| clean_gps(t).map(|(x, y)| (x, y + 5.0 * (t / 60.0).min(1.0)));
    drive(&mut c, 0.0, 60.0, good_ndt, drift);
    let run = drive(&mut c, 60.0, 65.0, |_| None, drift);
    assert_eq!(run.gps_passed, 0);
    let last = &run.ticks.last().unwrap().report;
    assert_eq!(last.source, Source::Hold);
    assert_eq!(last.level, Level::Error);
    assert!(
        run.ticks.iter().all(|t| t.seed.is_none()),
        "no seed from a GPS judged wrong"
    );
}

#[test]
fn a_gps_that_jumps_is_latched_until_it_agrees_again() {
    let mut c = checker();
    drive(&mut c, 0.0, 10.0, good_ndt, clean_gps);
    let jumped = |t: f64| clean_gps(t).map(|(x, y)| (x, y + 20.0));
    let run = drive(&mut c, 10.0, 20.0, good_ndt, jumped);
    assert!(matches!(
        run.ticks[1].report.gps,
        Some(GpsDoubt::Jumped { metres }) if metres > 19.0
    ));
    assert!(run.ticks.iter().all(|t| t.report.source == Source::Ndt));
    // Mended: the jump back is a jump too, then 5 s of agreement clear it.
    let run = drive(&mut c, 20.0, 35.0, good_ndt, clean_gps);
    assert_eq!(run.ticks.last().unwrap().report.gps, None);
    assert_eq!(run.ticks.last().unwrap().report.level, Level::Ok);
}

#[test]
fn a_silent_gps_changes_nothing_while_ndt_tracks() {
    let mut c = checker();
    drive(&mut c, 0.0, 10.0, good_ndt, clean_gps);
    let run = drive(&mut c, 10.0, 40.0, good_ndt, |_| None);
    assert!(run.ticks.iter().all(|t| t.report.source == Source::Ndt));
    let last = &run.ticks.last().unwrap().report;
    assert!(matches!(last.gps, Some(GpsDoubt::Stale { .. })));
    assert_eq!(last.level, Level::Warn);
    assert!(
        last.message.starts_with("NDT: GPS silent"),
        "{}",
        last.message
    );
}

#[test]
fn ndt_lost_hands_over_to_a_healthy_gps_and_seeds_ndt_from_it() {
    let mut c = checker();
    drive(&mut c, 0.0, 10.0, good_ndt, clean_gps);
    let run = drive(&mut c, 10.0, 21.0, |_| None, clean_gps);
    let first = &run.ticks[0];
    assert_eq!(first.report.source, Source::Gps);
    let handover = first.handover.as_ref().expect("a hand-over");
    assert_eq!((handover.from, handover.to), (Source::Ndt, Source::Gps));
    assert!(run.gps_passed > 20);
    let seeds: Vec<_> = run.ticks.iter().filter_map(|t| t.seed).collect();
    assert_eq!(
        seeds.len(),
        3,
        "one at once, then one every reseed_every (5 s)"
    );
    assert_eq!(seeds[0].from, SeedFrom::Gps);
    assert!((seeds[0].pose.x - 10.0).abs() < 0.5);
}

#[test]
fn ndt_lost_is_seeded_from_amcl_when_amcl_has_converged() {
    let mut c = checker();
    drive(&mut c, 0.0, 10.0, good_ndt, clean_gps);
    c.amcl(10.0, Pose2::new(10.02, 0.01, 0.0), Cov2::round(0.08), 0.01);
    c.ndt_status(10.0, NdtState::Lost, None);
    let tick = c.tick(10.05);
    assert_eq!(tick.seed.map(|s| s.from), Some(SeedFrom::Amcl));
    assert!(tick.report.message.contains("seeded from AMCL"));
}

#[test]
fn amcl_not_converged_falls_back_to_the_gps_seed() {
    let mut c = checker();
    drive(&mut c, 0.0, 10.0, good_ndt, clean_gps);
    c.amcl(10.0, Pose2::new(12.0, 1.0, 0.0), Cov2::round(1.5), 0.01);
    c.ndt_status(10.0, NdtState::Lost, None);
    assert_eq!(c.tick(10.05).seed.map(|s| s.from), Some(SeedFrom::Gps));
}

#[test]
fn ndt_back_on_track_takes_the_lead_again() {
    let mut c = checker();
    drive(&mut c, 0.0, 10.0, good_ndt, clean_gps);
    drive(&mut c, 10.0, 12.0, |_| None, clean_gps);
    let run = drive(&mut c, 12.0, 14.0, good_ndt, clean_gps);
    assert_eq!(run.ticks[0].report.source, Source::Ndt);
    assert_eq!(
        c.tick(14.0).report.handovers,
        3,
        "hold to NDT, NDT to GPS, GPS to NDT"
    );
}

#[test]
fn a_weak_ndt_that_disagrees_with_a_healthy_gps_is_judged_wrong() {
    let mut c = checker();
    // NDT tracking 4 m off the truth, on a score under trust_score (2.3): the wrong wall
    // (Ravensim's lot, 19 Nov: NDT 5 m off scored 0-2.0).
    let wrong = |t: f64| Some((Pose2::new(t, 4.0, 0.0), 1.2));
    let run = drive(&mut c, 0.0, 10.0, wrong, clean_gps);
    assert!(matches!(c.verdict(), Verdict::NdtWrong { .. }));
    let last = run.ticks.last().unwrap();
    assert_eq!(last.report.source, Source::Gps);
    assert!(matches!(last.report.ndt, Some(NdtDoubt::Disagrees { .. })));
    assert!(run.gps_passed > 0);
    assert!(run
        .ticks
        .iter()
        .any(|t| t.seed.map(|s| s.from) == Some(SeedFrom::Gps)));
    // Seeded, NDT finds the right place: they agree, and NDT leads again.
    let run = drive(&mut c, 10.0, 20.0, good_ndt, clean_gps);
    assert_eq!(c.verdict(), Verdict::Agree);
    assert_eq!(run.ticks.last().unwrap().report.source, Source::Ndt);
}

#[test]
fn both_in_doubt_means_hold() {
    let mut c = checker();
    drive(&mut c, 0.0, 5.0, good_ndt, clean_gps);
    let run = drive(&mut c, 5.0, 10.0, |_| None, |_| None);
    let last = &run.ticks.last().unwrap().report;
    assert_eq!(last.source, Source::Hold);
    assert_eq!(last.level, Level::Error);
    assert!(
        last.message.starts_with("HOLD: NDT lost, GPS silent"),
        "{}",
        last.message
    );
}

#[test]
fn at_the_start_it_holds_until_one_is_there() {
    let mut c = checker();
    let tick = c.tick(0.0);
    assert_eq!(tick.report.source, Source::Hold);
    assert_eq!(tick.report.ndt, Some(NdtDoubt::Never));
    assert_eq!(tick.report.gps, Some(GpsDoubt::Never));
    assert!(
        tick.seed.is_none(),
        "nothing to seed from, and NDT isn't lost"
    );
}

#[test]
fn a_gps_without_a_fix_or_with_nan_is_not_used() {
    let mut c = checker();
    drive(&mut c, 0.0, 5.0, |_| None, clean_gps);
    assert_eq!(c.tick(5.0).report.source, Source::Gps);
    c.gps_fix(false);
    assert!(!c.gps(5.1, 5.1, 0.0, GPS_COV));
    assert_eq!(c.tick(5.1).report.gps, Some(GpsDoubt::NoFix));
    assert!(!c.gps(5.3, f64::NAN, 0.0, GPS_COV));
    assert_eq!(c.tick(5.3).report.source, Source::Hold);
}

#[test]
fn a_gps_unsure_of_itself_is_not_used() {
    let mut c = checker();
    c.gps_fix(true);
    assert!(!c.gps(0.0, 0.0, 0.0, Cov2::round(4.0)));
    assert!(matches!(
        c.tick(0.0).report.gps,
        Some(GpsDoubt::Inaccurate { .. })
    ));
}

#[test]
fn bad_settings_are_refused() {
    let s = Settings {
        agree_m: 2.0,
        ..Settings::default()
    };
    assert!(matches!(
        Checker::new(s),
        Err(SettingsError::NoHysteresis { .. })
    ));
    let s = Settings {
        window: 0.0,
        ..Settings::default()
    };
    assert_eq!(
        Checker::new(s).unwrap_err(),
        SettingsError::NotPositive {
            name: "window",
            value: 0.0
        }
    );
}

#[test]
fn ndt_status_words_are_read() {
    assert_eq!(NdtState::from_message("TRACKING"), Some(NdtState::Tracking));
    assert_eq!(
        NdtState::from_message("LOST: 5 poor fits"),
        Some(NdtState::Lost)
    );
    assert_eq!(
        NdtState::from_message("WAITING for a start"),
        Some(NdtState::Waiting)
    );
    assert_eq!(NdtState::from_message(""), None);
}

#[test]
fn a_spread_is_its_largest_axis() {
    assert!((Cov2::new(4.0, 0.0, 1.0).std_max() - 2.0).abs() < 1e-12);
    assert!((Cov2::round(0.5).std_max() - 0.5).abs() < 1e-12);
}
