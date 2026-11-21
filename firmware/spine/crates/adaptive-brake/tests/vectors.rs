//! The adaptive brake against the examples the Python checks too
//! (`firmware/spine/adaptive_brake_vectors.txt`, from `gen_adaptive_brake_vectors.py`):
//! the car's copy and the sim's must send the same brake, tick by tick (TODO 258).

use adaptive_brake::{AdaptiveBrake, BrakeLimits, CarModel, Tuning};

const VECTORS: &str = include_str!("../../../adaptive_brake_vectors.txt");

/// Both are f64; only `exp` (libm's against the C library's) may differ, in the last bit.
const TOLERANCE: f64 = 1e-9;

fn numbers(fields: &[&str], line: &str) -> Vec<f64> {
    fields
        .iter()
        .map(|f| {
            f.parse()
                .unwrap_or_else(|_| panic!("a number, not {f:?}, in {line:?}"))
        })
        .collect()
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= TOLERANCE * (1.0 + b.abs())
}

#[test]
fn the_rust_sends_what_the_python_sends() {
    let (mut car, mut limits) = (None, None);
    let mut brake: Option<AdaptiveBrake> = None;
    let mut name = String::new();
    let (mut cases, mut ticks) = (0, 0);
    for line in VECTORS.lines() {
        let fields: Vec<&str> = line.split_whitespace().collect();
        let Some((&word, rest)) = fields.split_first() else {
            continue;
        };
        match word {
            w if w.starts_with('#') => {}
            "case" => {
                name = rest.join(" ");
                cases += 1;
                brake = None;
            }
            "car" => {
                let n = numbers(rest, line);
                car = Some(CarModel {
                    max_brake: n[0],
                    brake_lag: n[1],
                    brake_dead_time: n[2],
                    drag: n[3],
                    throttle_dead_time: n[4],
                    throttle_lag: n[5],
                });
            }
            "limits" => {
                let n = numbers(rest, line);
                limits = Some(BrakeLimits::new(n[0], n[1], n[2], n[3], n[4]).expect(line));
            }
            "tuning" => {
                let n = numbers(rest, line);
                let tuning = Tuning {
                    filter_time: n[0],
                    adapt_time: n[1],
                    min_decel: n[2],
                    min_speed: n[3],
                    max_gain: n[4],
                    rise_rate: n[5],
                    high_share: n[6],
                };
                brake = Some(AdaptiveBrake::new(
                    car.expect("car before tuning"),
                    limits.expect("limits before tuning"),
                    tuning,
                ));
            }
            "reset" => brake.as_mut().expect(line).reset(),
            "reset_stop" => brake.as_mut().expect(line).reset_stop(),
            "tick" => {
                let b = brake.as_mut().expect(line);
                let speed = match rest[2] {
                    "none" => None,
                    s => Some(numbers(&[s], line)[0]),
                };
                let n = numbers(&[rest[0], rest[1], rest[3]], line);
                let out = b.update(n[0], n[1], speed, n[2]).expect(line);
                assert_eq!(rest[4], "->", "{line}");
                let want = numbers(&rest[5..8], line);
                assert!(
                    close(out.brake, want[0])
                        && close(out.gain, want[1])
                        && close(out.effectiveness, want[2]),
                    "{name}: {line}\n  rust gave brake {} gain {} effectiveness {}",
                    out.brake,
                    out.gain,
                    out.effectiveness
                );
                assert_eq!(
                    (out.capped, out.spent),
                    (rest[8] == "1", rest[9] == "1"),
                    "{name}: {line}"
                );
                ticks += 1;
            }
            other => panic!("unknown line {other:?}: {line}"),
        }
    }
    assert_eq!(cases, 6, "every case read");
    assert!(ticks > 1000, "only {ticks} ticks");
}
