//! The navigator against a pretend planner and follower: the trees' paths through it.

use navigator::{
    Abort, BackUp, Command, FollowError, Mission, Navigator, Outcome, PlanError, Pose, Settings,
    WaypointMode,
};

fn straight(to: &Pose) -> Vec<Pose> {
    (0..=10)
        .map(|i| Pose::new(to.x * i as f64 / 10.0, to.y * i as f64 / 10.0, to.yaw))
        .collect()
}

fn plan_token(out: &[Command]) -> (u64, Vec<Pose>) {
    out.iter()
        .find_map(|c| match c {
            Command::Plan { token, goals } => Some((*token, goals.clone())),
            _ => None,
        })
        .unwrap_or_else(|| panic!("no plan in {out:?}"))
}

fn follow_token(out: &[Command]) -> u64 {
    out.iter()
        .find_map(|c| match c {
            Command::Follow { token, .. } => Some(*token),
            _ => None,
        })
        .unwrap_or_else(|| panic!("no follow in {out:?}"))
}

fn finished(out: &[Command]) -> Option<(u64, Outcome)> {
    out.iter().find_map(|c| match c {
        Command::Finished { id, outcome } => Some((*id, outcome.clone())),
        _ => None,
    })
}

const GOAL: Pose = Pose::new(30.0, 0.0, 0.0);

/// The trees as until 15 Nov: a new path every second (no KeepAValidPath), to test the replans.
fn replanning() -> Settings {
    Settings {
        keep_path_age: 0.0,
        ..Settings::default()
    }
}

#[test]
fn plan_follow_arrive() {
    let mut nav = Navigator::new(replanning());
    assert!(nav.start(1, Mission::ToPose(GOAL), 0.0).is_empty());
    let (t, goals) = plan_token(&nav.tick(0.0, Some(&Pose::default())));
    assert_eq!(goals, vec![GOAL]);
    let f = follow_token(&nav.planned(t, Ok(straight(&GOAL)), 0.1));
    // Nothing new before the second is up; then a replan, the follower gets it.
    assert!(nav.tick(0.5, Some(&Pose::default())).is_empty());
    let (t2, _) = plan_token(&nav.tick(1.0, Some(&Pose::new(1.0, 0.0, 0.0))));
    let f2 = follow_token(&nav.planned(t2, Ok(straight(&GOAL)), 1.1));
    // The first follow goal's answer is late now: ignored.
    assert!(nav
        .followed(f, Err((FollowError::Unknown, "replaced".into())), 1.2)
        .is_empty());
    let out = nav.followed(f2, Ok(()), 20.0);
    assert_eq!(
        finished(&out),
        Some((1, Outcome::Succeeded { missed: vec![] }))
    );
    assert_eq!(nav.goal(), None);
}

#[test]
fn final_approach_keeps_the_path() {
    let mut nav = Navigator::new(Settings::default());
    nav.start(1, Mission::ToPose(GOAL), 0.0);
    let (t, _) = plan_token(&nav.tick(0.0, Some(&Pose::default())));
    nav.planned(t, Ok(straight(&GOAL)), 0.0);
    // 5 m out: no replan for up to 20 s.
    let near = Pose::new(25.0, 0.5, 0.0);
    for now in [3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 19.0] {
        nav.follower_heard(now); // the follower's feedback, every tick in a real run
        assert!(nav.tick(now, Some(&near)).is_empty(), "{now}");
    }
    // The path is too old now: plan again.
    nav.follower_heard(21.0);
    plan_token(&nav.tick(21.0, Some(&near)));
}

/// The trees' KeepAValidPath (TODO 168): far from the goal too, a path that still ends at
/// the goal is kept up to 10 s while IsPathValid says it is clear; a blocked one, or an
/// older one, is planned again. Planning every second moved the path a few cm each time
/// and the car swayed twice as much (a left turn without ROS: 0.075 against 0.038 m/s²).
#[test]
fn a_valid_path_is_kept_for_10_s() {
    let mut nav = Navigator::new(Settings::default());
    nav.start(1, Mission::ToPose(GOAL), 0.0);
    let (t, _) = plan_token(&nav.tick(0.0, Some(&Pose::default())));
    nav.planned(t, Ok(straight(&GOAL)), 0.0);
    let far = Pose::new(5.0, 0.0, 0.0);
    for now in [1.0, 3.0, 6.0, 9.5] {
        nav.follower_heard(now);
        let (made, _) = nav
            .path_to_check(now, Some(&far))
            .expect("a kept path is checked");
        nav.path_checked(made, true);
        assert!(nav.tick(now, Some(&far)).is_empty(), "kept at {now} s");
    }
    // Ten seconds old: a new one.
    nav.follower_heard(10.0);
    let (t2, _) = plan_token(&nav.tick(10.0, Some(&far)));
    nav.planned(t2, Ok(straight(&GOAL)), 10.1);
    // Something on it: planned again at the next second.
    nav.follower_heard(11.2);
    let (made, _) = nav.path_to_check(11.2, Some(&far)).expect("checked");
    nav.path_checked(made, false);
    plan_token(&nav.tick(11.2, Some(&far)));
}

#[test]
fn through_poses_plan_again_every_period() {
    let mut nav = Navigator::new(Settings::default());
    let via = Pose::new(15.0, 0.0, 0.0);
    nav.start(1, Mission::ThroughPoses(vec![via, GOAL]), 0.0);
    let (t, _) = plan_token(&nav.tick(0.0, Some(&Pose::default())));
    nav.planned(t, Ok(straight(&GOAL)), 0.0);
    let far = Pose::new(2.0, 0.0, 0.0);
    nav.follower_heard(3.0);
    assert!(nav.path_to_check(3.0, Some(&far)).is_none());
    plan_token(&nav.tick(3.0, Some(&far)));
}

#[test]
fn a_blocked_path_on_the_final_approach_is_planned_again() {
    let mut nav = Navigator::new(replanning());
    nav.start(1, Mission::ToPose(GOAL), 0.0);
    let (t, _) = plan_token(&nav.tick(0.0, Some(&Pose::default())));
    nav.planned(t, Ok(straight(&GOAL)), 0.0);
    let far = Pose::new(5.0, 0.0, 0.0);
    let near = Pose::new(25.0, 0.5, 0.0);
    // Far from the goal there is nothing to ask: the replans check the way.
    assert!(nav.path_to_check(0.5, Some(&far)).is_none());
    nav.follower_heard(3.0);
    let (made, poses) = nav
        .path_to_check(3.0, Some(&near))
        .expect("on the final approach");
    assert_eq!(poses.len(), 11);
    nav.path_checked(made, true);
    assert!(
        nav.tick(3.0, Some(&near)).is_empty(),
        "a clear path is kept"
    );
    // An answer about some other path changes nothing.
    nav.path_checked(made + 1.0, false);
    nav.follower_heard(4.0);
    assert!(nav.tick(4.0, Some(&near)).is_empty());
    // Blocked: plan again at once.
    nav.path_checked(made, false);
    nav.follower_heard(5.0);
    plan_token(&nav.tick(5.0, Some(&near)));
}

#[test]
fn a_failed_replan_while_driving_keeps_going_then_recovers() {
    let mut nav = Navigator::new(replanning());
    nav.start(1, Mission::ToPose(GOAL), 0.0);
    let (t, _) = plan_token(&nav.tick(0.0, None));
    nav.planned(t, Ok(straight(&GOAL)), 0.0);
    let mut now = 1.0;
    for _ in 0..3 {
        nav.follower_heard(now);
        let (t, _) = plan_token(&nav.tick(now, None));
        assert!(nav
            .planned(t, Err((PlanError::NoValidPath, "no path".into())), now)
            .is_empty());
        now += 1.0;
    }
    // The fourth in a row: the planner's own recovery, the global costmap cleared.
    nav.follower_heard(now);
    let (t, _) = plan_token(&nav.tick(now, None));
    let out = nav.planned(t, Err((PlanError::NoValidPath, "no path".into())), now);
    assert!(matches!(
        out[..],
        [Command::Clear {
            local: false,
            global: true,
            ..
        }]
    ));
}

#[test]
fn the_follower_failing_clears_local_then_recoveries_in_turn_then_gives_up() {
    let settings = Settings {
        retries: 2,
        ..Settings::default()
    };
    let mut nav = Navigator::new(settings);
    nav.start(7, Mission::ToPose(GOAL), 0.0);
    let (t, _) = plan_token(&nav.tick(0.0, None));
    let f = follow_token(&nav.planned(t, Ok(straight(&GOAL)), 0.0));
    // No progress: clear the local costmap, follow the same path again.
    let out = nav.followed(
        f,
        Err((FollowError::FailedToMakeProgress, "no progress".into())),
        10.0,
    );
    let Command::Clear {
        token,
        local: true,
        global: false,
    } = out[0]
    else {
        panic!("{out:?}")
    };
    let f = follow_token(&nav.recovered(token, true, 10.1));
    // Again: the first outer recovery, both costmaps.
    let out = nav.followed(
        f,
        Err((FollowError::FailedToMakeProgress, "no progress".into())),
        20.0,
    );
    assert_eq!(out[0], Command::StopFollowing);
    let Command::Clear {
        token,
        local: true,
        global: true,
    } = out[1]
    else {
        panic!("{out:?}")
    };
    // After it, a new plan at once.
    let (t, _) = plan_token(&nav.recovered(token, true, 20.5));
    let f = follow_token(&nav.planned(t, Ok(straight(&GOAL)), 20.6));
    let out = nav.followed(
        f,
        Err((FollowError::FailedToMakeProgress, "no progress".into())),
        30.0,
    );
    let out = if matches!(out[..], [Command::Clear { .. }]) {
        // The follower's own clear comes back each attempt.
        let Command::Clear { token, .. } = out[0] else {
            unreachable!()
        };
        let f = follow_token(&nav.recovered(token, true, 30.1));
        nav.followed(
            f,
            Err((FollowError::FailedToMakeProgress, "no progress".into())),
            31.0,
        )
    } else {
        out
    };
    // The second outer recovery: wait 5 s (no back up: forward only).
    assert_eq!(out, vec![Command::StopFollowing]);
    assert!(nav.tick(33.0, None).is_empty());
    let (t, _) = plan_token(&nav.tick(36.1, None));
    let f = follow_token(&nav.planned(t, Ok(straight(&GOAL)), 36.2));
    let out = nav.followed(
        f,
        Err((FollowError::FailedToMakeProgress, "no progress".into())),
        40.0,
    );
    let Command::Clear { token, .. } = out[0] else {
        panic!("{out:?}")
    };
    let f = follow_token(&nav.recovered(token, true, 40.1));
    // Out of retries: aborted with the follower's code.
    let out = nav.followed(
        f,
        Err((FollowError::FailedToMakeProgress, "no progress".into())),
        41.0,
    );
    match finished(&out) {
        Some((7, Outcome::Aborted { reason, .. })) => assert_eq!(reason.code(), 105),
        other => panic!("{other:?}"),
    }
}

/// TODO 174: a narrow street blocked ahead. The car stands before the block (the stop
/// zone holds it) and makes no progress; the navigator clears the local costmap, follows
/// again, gives up on that path, clears both and plans again; the planner, seeing the
/// block, backs out of the street (a path with a cusp: lattice-planner's scene "blocked
/// narrow street"); the follower gets it as planned, cusp and all, and arrives.
#[test]
fn a_blocked_narrow_street_is_left_backwards_on_the_next_plan() {
    let goal = Pose::new(40.0, 5.0, 0.0);
    let into_street: Vec<Pose> = (0..=20)
        .map(|i| Pose::new(19.75, 10.0 + f64::from(i), std::f64::consts::FRAC_PI_2))
        .collect();
    // Backwards down the street (the poses still face north), a cusp, forward east.
    let mut out_of_it: Vec<Pose> = (0..=17)
        .map(|i| Pose::new(19.75, 22.0 - f64::from(i), std::f64::consts::FRAC_PI_2))
        .collect();
    out_of_it.extend((0..=20).map(|i| Pose::new(20.0 + f64::from(i), 5.0, 0.0)));
    let mut nav = Navigator::new(Settings {
        retries: 3,
        ..Settings::default()
    });
    nav.start(4, Mission::ToPose(goal), 0.0);
    let (t, _) = plan_token(&nav.tick(0.0, None));
    let f = follow_token(&nav.planned(t, Ok(into_street), 0.0));
    // Held before the block: no progress. The local costmap cleared, the same path again.
    let out = nav.followed(
        f,
        Err((FollowError::FailedToMakeProgress, "no progress".into())),
        10.0,
    );
    let Command::Clear { token, .. } = out[0] else {
        panic!("{out:?}")
    };
    let f = follow_token(&nav.recovered(token, true, 10.1));
    let out = nav.followed(
        f,
        Err((FollowError::FailedToMakeProgress, "no progress".into())),
        20.0,
    );
    let Some(Command::Clear { token, .. }) = out.get(1) else {
        panic!("{out:?}")
    };
    // Planned again: the block is in the costmap now, so the path backs out.
    let (t, _) = plan_token(&nav.recovered(*token, true, 20.5));
    let out = nav.planned(t, Ok(out_of_it.clone()), 20.6);
    let Some(Command::Follow { token: f, path }) = out
        .iter()
        .find(|c| matches!(c, Command::Follow { .. }))
        .cloned()
    else {
        panic!("{out:?}")
    };
    assert_eq!(
        path, out_of_it,
        "the follower gets the reversing path as planned"
    );
    assert_eq!(
        finished(&nav.followed(f, Ok(()), 60.0)),
        Some((4, Outcome::Succeeded { missed: vec![] }))
    );
}

#[test]
fn back_up_is_a_recovery_only_where_the_car_may_reverse() {
    let settings = Settings {
        backup: Some(BackUp {
            distance: 1.0,
            speed: 0.25,
        }),
        ..Settings::default()
    };
    let mut nav = Navigator::new(settings);
    nav.start(1, Mission::ToPose(GOAL), 0.0);
    let mut kinds = Vec::new();
    let mut now = 0.0;
    // An outer recovery that answers plans again at once; a wait, at the next tick.
    let mut pending: Vec<Command> = Vec::new();
    for _ in 0..3 {
        let asked = if pending.is_empty() {
            nav.tick(now, None)
        } else {
            std::mem::take(&mut pending)
        };
        let (t, _) = plan_token(&asked);
        // A goal the planner can't reach, and clearing doesn't help.
        let out = nav.planned(t, Err((PlanError::NoValidPath, "no path".into())), now);
        let Command::Clear { token, .. } = out[0] else {
            panic!("{out:?}")
        };
        let out = nav.recovered(token, true, now);
        let (t, _) = plan_token(&out);
        let out = nav.planned(t, Err((PlanError::NoValidPath, "no path".into())), now);
        kinds.push(match out.get(1) {
            Some(Command::Clear { .. }) => "clear",
            Some(Command::BackUp { .. }) => "backup",
            None => "wait",
            other => panic!("{other:?}"),
        });
        match out.get(1) {
            Some(Command::Clear { token, .. }) | Some(Command::BackUp { token, .. }) => {
                pending = nav.recovered(*token, true, now);
            }
            _ => now += 5.0,
        }
        now += 0.1;
    }
    assert_eq!(kinds, vec!["clear", "wait", "backup"]);
}

#[test]
fn errors_no_recovery_can_fix_end_the_goal_at_once() {
    let mut nav = Navigator::new(Settings::default());
    nav.start(3, Mission::ToPose(GOAL), 0.0);
    let (t, _) = plan_token(&nav.tick(0.0, None));
    let out = nav.planned(
        t,
        Err((
            PlanError::GoalOutsideMap,
            "the goal is off the costmap".into(),
        )),
        0.0,
    );
    match finished(&out) {
        Some((3, Outcome::Aborted { reason, .. })) => {
            assert_eq!(reason.code(), 204);
            let message = reason.to_string();
            assert!(message.contains("off the costmap"), "{message}")
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_new_goal_replaces_the_old_and_cancel_stops() {
    let mut nav = Navigator::new(Settings::default());
    nav.start(1, Mission::ToPose(GOAL), 0.0);
    let (t, _) = plan_token(&nav.tick(0.0, None));
    let out = nav.start(2, Mission::ToPose(Pose::new(5.0, 5.0, 0.0)), 0.5);
    assert!(matches!(
        finished(&out),
        Some((
            1,
            Outcome::Aborted {
                reason: Abort::Replaced,
                ..
            }
        ))
    ));
    // The old goal's plan comes back late: ignored.
    assert!(nav.planned(t, Ok(straight(&GOAL)), 0.6).is_empty());
    let out = nav.cancel(2);
    assert_eq!(out[0], Command::StopFollowing);
    assert_eq!(finished(&out), Some((2, Outcome::Cancelled)));
    assert!(nav.cancel(2).is_empty());
}

#[test]
fn through_poses_plans_through_all_and_drops_the_passed() {
    let poses = vec![Pose::new(10.0, 1.5, 0.0), Pose::new(20.0, 0.0, 0.0), GOAL];
    let mut nav = Navigator::new(Settings::default());
    nav.start(1, Mission::ThroughPoses(poses.clone()), 0.0);
    let (t, goals) = plan_token(&nav.tick(0.0, Some(&Pose::default())));
    assert_eq!(goals, poses);
    nav.planned(t, Ok(straight(&GOAL)), 0.0);
    // Every 3 s; the first pose (1.5 m off the path) is passed along it at x = 11.
    assert!(nav.tick(2.0, Some(&Pose::new(11.0, 0.0, 0.0))).is_empty());
    let (_, goals) = plan_token(&nav.tick(3.0, Some(&Pose::new(11.0, 0.0, 0.0))));
    assert_eq!(goals, poses[1..].to_vec());
    let p = nav.progress(Some(&Pose::new(11.0, 0.0, 0.0))).unwrap();
    assert_eq!((p.waypoint, p.poses_remaining), (1, 2));
    assert!((p.distance_remaining - 19.0).abs() < 1e-6);
}

#[test]
fn waypoints_pass_through_by_default_or_stop_at_each() {
    let wps = vec![Pose::new(10.0, 0.0, 0.0), GOAL];
    let mut nav = Navigator::new(Settings::default());
    nav.start(1, Mission::Waypoints(wps.clone()), 0.0);
    let (_, goals) = plan_token(&nav.tick(0.0, None));
    assert_eq!(goals, wps, "one plan through both");

    let mut nav = Navigator::new(Settings {
        waypoints: WaypointMode::StopAtEach,
        ..Settings::default()
    });
    nav.start(1, Mission::Waypoints(wps.clone()), 0.0);
    let (t, goals) = plan_token(&nav.tick(0.0, None));
    assert_eq!(goals, vec![wps[0]]);
    let f = follow_token(&nav.planned(t, Ok(straight(&wps[0])), 0.0));
    assert!(nav.followed(f, Ok(()), 10.0).is_empty());
    let (t, goals) = plan_token(&nav.tick(10.0, None));
    assert_eq!(goals, vec![wps[1]]);
    // The second can't be planned at all: missed, and the goal ends (it was the last).
    let out = nav.planned(
        t,
        Err((PlanError::GoalOccupied, "goal occupied".into())),
        10.0,
    );
    match finished(&out) {
        Some((1, Outcome::Aborted { missed, .. })) => assert_eq!(missed, vec![1]),
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_start_marked_occupied_clears_the_global_costmap_first() {
    let mut nav = Navigator::new(Settings::default());
    nav.start(1, Mission::ToPose(GOAL), 0.0);
    let (t, _) = plan_token(&nav.tick(0.0, None));
    let out = nav.planned(
        t,
        Err((PlanError::StartOccupied, "start occupied".into())),
        0.0,
    );
    assert!(matches!(
        out[..],
        [Command::Clear {
            local: false,
            global: true,
            ..
        }]
    ));
}

#[test]
fn empty_goals_are_refused() {
    let mut nav = Navigator::new(Settings::default());
    let out = nav.start(4, Mission::ThroughPoses(vec![]), 0.0);
    assert!(matches!(
        finished(&out),
        Some((
            4,
            Outcome::Aborted {
                reason: Abort::InvalidGoal(_),
                ..
            }
        ))
    ));
    let out = nav.start(5, Mission::ToPose(Pose::new(f64::NAN, 0.0, 0.0)), 0.0);
    assert!(finished(&out).is_some());
    assert_eq!(nav.goal(), None);
}

#[test]
fn error_codes_are_nav2s_both_ways() {
    for code in 200..=208 {
        assert_eq!(PlanError::from_code(code).code(false), code);
    }
    for code in 300..=309 {
        assert_eq!(PlanError::from_code(code).code(true), code);
    }
    for code in 100..=107 {
        assert_eq!(FollowError::from_code(code).code(), code);
    }
    // Numbers that aren't Nav2's (0 too: a server that failed without saying why).
    assert_eq!(PlanError::from_code(0), PlanError::Unknown);
    assert_eq!(PlanError::from_code(209), PlanError::Unknown);
    assert_eq!(FollowError::from_code(0), FollowError::Unknown);
    assert_eq!(FollowError::from_code(999), FollowError::Unknown);
    // A failed through-poses plan reports the 300s.
    let poses = vec![Pose::new(10.0, 0.0, 0.0), GOAL];
    let mut nav = Navigator::new(Settings::default());
    nav.start(1, Mission::ThroughPoses(poses), 0.0);
    let (t, _) = plan_token(&nav.tick(0.0, None));
    let out = nav.planned(t, Err((PlanError::GoalOutsideMap, String::new())), 0.0);
    match finished(&out) {
        Some((1, Outcome::Aborted { reason, .. })) => assert_eq!(reason.code(), 304),
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_planner_that_never_answers_times_out_and_is_recovered() {
    let mut nav = Navigator::new(Settings::default());
    nav.start(1, Mission::ToPose(GOAL), 0.0);
    let (t, _) = plan_token(&nav.tick(0.0, None));
    // Nothing until plan_timeout (10 s).
    assert!(nav.tick(9.9, None).is_empty());
    // Then a planner timeout (207): recoverable, so the global costmap is cleared first.
    let out = nav.tick(10.0, None);
    assert!(
        matches!(
            out[..],
            [Command::Clear {
                local: false,
                global: true,
                ..
            }]
        ),
        "{out:?}"
    );
    // Its answer, if it ever comes, is late.
    assert!(nav.planned(t, Ok(straight(&GOAL)), 10.5).is_empty());
}

#[test]
fn a_dead_stack_ends_the_goal_in_bounded_time() {
    let settings = Settings::default();
    let mut nav = Navigator::new(settings.clone());
    nav.start(1, Mission::ToPose(GOAL), 0.0);
    // Nobody answers anything: plans, clears and waits all run into their limits.
    let mut now = 0.0;
    let mut end = None;
    while end.is_none() && now < 600.0 {
        end = finished(&nav.tick(now, None));
        now += 0.1;
    }
    let Some((1, Outcome::Aborted { reason, .. })) = end else {
        panic!("still running after 600 s: {end:?}");
    };
    // The cause, not the clear that failed after it.
    assert_eq!(reason.code(), 207, "{reason}");
    // Each of the 7 attempts (6 recoveries): a plan, its clear, then a recovery.
    let per_attempt = settings.plan_timeout + 2.0 * settings.recovery_timeout.max(settings.wait);
    assert!(now <= 7.0 * per_attempt + 1.0, "{now} s");
}

#[test]
fn a_silent_follower_fails_the_goal_with_controller_timed_out() {
    let mut nav = Navigator::new(Settings::default());
    nav.start(1, Mission::ToPose(GOAL), 0.0);
    let (t, _) = plan_token(&nav.tick(0.0, None));
    let f = follow_token(&nav.planned(t, Ok(straight(&GOAL)), 0.0));
    // Feedback keeps it alive past follow_timeout (4 s), replans or not.
    let mut now = 0.0;
    while now < 10.0 {
        now += 0.5;
        nav.follower_heard(now);
        for c in nav.tick(now, None) {
            if let Command::Plan { token, .. } = c {
                nav.planned(token, Ok(straight(&GOAL)), now);
            }
        }
        assert_eq!(nav.goal(), Some(1));
    }
    // Then nothing: still replanning every second (new paths), but the follower never
    // speaks again.
    let silent_from = now;
    let mut end = None;
    while end.is_none() && now < 30.0 {
        now += 0.1;
        for c in nav.tick(now, None) {
            match c {
                Command::Plan { token, .. } => {
                    nav.planned(token, Ok(straight(&GOAL)), now);
                }
                Command::Finished { outcome, .. } => end = Some(outcome),
                _ => {}
            }
        }
    }
    match end {
        Some(Outcome::Aborted { reason, .. }) => {
            assert_eq!(reason.code(), FollowError::ControllerTimedOut.code());
        }
        other => panic!("{other:?}"),
    }
    assert!(now - silent_from <= 4.0 + 0.11, "{} s", now - silent_from);
    // The follower's late answer changes nothing.
    assert!(nav.followed(f, Ok(()), now).is_empty());
}

#[test]
fn a_clear_that_never_answers_counts_as_failed() {
    let mut nav = Navigator::new(Settings::default());
    nav.start(1, Mission::ToPose(GOAL), 0.0);
    let (t, _) = plan_token(&nav.tick(0.0, None));
    let f = follow_token(&nav.planned(t, Ok(straight(&GOAL)), 0.0));
    let out = nav.followed(
        f,
        Err((FollowError::FailedToMakeProgress, String::new())),
        1.0,
    );
    let Command::Clear { local: true, .. } = out[0] else {
        panic!("{out:?}")
    };
    assert!(nav.tick(5.9, None).is_empty());
    // recovery_timeout (5 s) on: the follower's own clear failed, it follows again.
    follow_token(&nav.tick(6.0, None));
}
