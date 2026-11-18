//! The planner from its settings: the JSON block `aav_description`'s
//! `vehicle.planner_params` writes (nav2.yaml's tuned Lattice block, or with
//! `"search": "hybrid"` its Hybrid-A* block; the car's outline and padding; the global
//! costmap's inflation). The node's `settings` parameter and `tools/diffusion_data.py`'s
//! teachers both read it here, so both plan the same way.

use serde_json::Value;

use costmap::{footprint, Inflation};

use crate::{Hybrid, Lattice, LatticeError, Planner, Settings, SmootherSettings};

fn wrong(text: String) -> LatticeError {
    LatticeError(text)
}

/// `block[key]` as a number; left out: `default`.
fn number(block: &Value, key: &str, default: f64) -> Result<f64, LatticeError> {
    match &block[key] {
        Value::Null => Ok(default),
        v => v
            .as_f64()
            .ok_or_else(|| wrong(format!("{key}: a number is needed"))),
    }
}

/// `block[key]` as true or false; left out: `default`.
fn flag(block: &Value, key: &str, default: bool) -> Result<bool, LatticeError> {
    match &block[key] {
        Value::Null => Ok(default),
        v => v
            .as_bool()
            .ok_or_else(|| wrong(format!("{key}: true or false"))),
    }
}

/// Nav2's smoother block (`<planner>.smoother`: w_data, w_smooth, max_iterations,
/// tolerance, do_refinement, refinement_num); a key left out keeps `d`'s. The launch's
/// plan_smoothing:=strong lowers w_data (vehicle.with_plan_smoothing, TODO 168).
fn smoother(block: &Value, d: SmootherSettings) -> Result<SmootherSettings, LatticeError> {
    if block.is_null() {
        return Ok(d);
    }
    let at = |key: &str| format!("smoother.{key}");
    let number = |key: &str, default: f64| {
        number(block, key, default).map_err(|_| wrong(format!("{}: a number is needed", at(key))))
    };
    let flag = |key: &str, default: bool| {
        flag(block, key, default).map_err(|_| wrong(format!("{}: true or false", at(key))))
    };
    Ok(SmootherSettings {
        tolerance: number("tolerance", d.tolerance)?,
        max_iterations: number("max_iterations", d.max_iterations as f64)?.max(0.0) as usize,
        w_data: number("w_data", d.w_data)?,
        w_smooth: number("w_smooth", d.w_smooth)?,
        do_refinement: flag("do_refinement", d.do_refinement)?,
        refinement_num: number("refinement_num", d.refinement_num as f64)?.max(0.0) as usize,
    })
}

/// The planner from vehicle.planner_params' JSON: the State Lattice (its primitives file
/// read from `lattice_filepath`), or Hybrid-A* when `search` is "hybrid".
///
/// # Errors
///
/// [`LatticeError`]: a key of the wrong type, a missing outline or radius, or a
/// primitives file that can't be read or used.
pub fn planner_from_json(spec: &Value) -> Result<Planner, LatticeError> {
    let hybrid = match spec["search"].as_str().unwrap_or("lattice") {
        "lattice" => false,
        "hybrid" => true,
        other => return Err(wrong(format!("search {other:?}: lattice or hybrid"))),
    };
    let number = |key: &str, default: f64| number(spec, key, default);
    let flag = |key: &str, default: bool| flag(spec, key, default);
    let d = if hybrid {
        Settings::hybrid()
    } else {
        Settings::default()
    };
    // Hybrid-A*'s motion model says whether it reverses (Nav2's motion_model_for_search);
    // the lattice's allow_reverse_expansion.
    let allow_reverse = if hybrid {
        match spec["motion_model_for_search"].as_str().unwrap_or("DUBIN") {
            "DUBIN" => false,
            "REEDS_SHEPP" => true,
            other => {
                return Err(wrong(format!(
                    "motion_model_for_search {other:?}: DUBIN or REEDS_SHEPP"
                )))
            }
        }
    } else {
        flag("allow_reverse_expansion", d.allow_reverse)?
    };
    let settings = Settings {
        allow_reverse,
        allow_unknown: flag("allow_unknown", d.allow_unknown)?,
        tolerance: number("tolerance", d.tolerance)?,
        max_iterations: number("max_iterations", d.max_iterations as f64)? as usize,
        max_on_approach_iterations: number(
            "max_on_approach_iterations",
            d.max_on_approach_iterations as f64,
        )? as usize,
        max_planning_time: number("max_planning_time", d.max_planning_time)?,
        terminal_checking_interval: number(
            "terminal_checking_interval",
            d.terminal_checking_interval as f64,
        )?
        .max(1.0) as usize,
        reverse_penalty: number("reverse_penalty", d.reverse_penalty)?,
        change_penalty: number("change_penalty", d.change_penalty)?,
        cusp_penalty: number("cusp_penalty", d.cusp_penalty)?,
        non_straight_penalty: number("non_straight_penalty", d.non_straight_penalty)?,
        cost_penalty: number("cost_penalty", d.cost_penalty)?,
        retrospective_penalty: number("retrospective_penalty", d.retrospective_penalty)?,
        rotation_penalty: number("rotation_penalty", d.rotation_penalty)?,
        analytic_expansion_ratio: number("analytic_expansion_ratio", d.analytic_expansion_ratio)?,
        analytic_expansion_max_length: number(
            "analytic_expansion_max_length",
            d.analytic_expansion_max_length,
        )?,
        analytic_expansion_max_cost: number(
            "analytic_expansion_max_cost",
            d.analytic_expansion_max_cost,
        )?,
        analytic_expansion_max_cost_override: flag(
            "analytic_expansion_max_cost_override",
            d.analytic_expansion_max_cost_override,
        )?,
        lookup_table_size: number("lookup_table_size", d.lookup_table_size)?,
        smooth_path: flag("smooth_path", d.smooth_path)?,
        smoother: smoother(&spec["smoother"], d.smoother)?,
    };
    let corners: Vec<(f64, f64)> = spec["footprint"]
        .as_array()
        .ok_or_else(|| wrong("footprint: a list of [x, y] is needed".into()))?
        .iter()
        .map(|p| Some((p[0].as_f64()?, p[1].as_f64()?)))
        .collect::<Option<_>>()
        .ok_or_else(|| wrong("footprint: [x, y] numbers".into()))?;
    let outline = footprint::padded(&corners, number("footprint_padding", 0.01)?);
    let inflation = match &spec["inflation"] {
        Value::Null => None,
        i => Some(Inflation {
            inflation_radius: i["radius"]
                .as_f64()
                .ok_or_else(|| wrong("inflation.radius".into()))?,
            cost_scaling_factor: i["scaling"]
                .as_f64()
                .ok_or_else(|| wrong("inflation.scaling".into()))?,
            inscribed_radius: footprint::radii(&outline).0,
        }),
    };
    if hybrid {
        let radius = number("minimum_turning_radius", f64::NAN)?;
        if radius.is_nan() || radius <= 0.0 {
            return Err(wrong(
                "minimum_turning_radius: a radius in m is needed (vehicle.plan_turning_radius)"
                    .into(),
            ));
        }
        let defaults = Hybrid::new(radius);
        let bins = number(
            "angle_quantization_bins",
            defaults.angle_quantization_bins as f64,
        )?;
        let spec_hybrid = Hybrid {
            minimum_turning_radius: radius,
            angle_quantization_bins: bins as usize,
            allow_primitive_interpolation: flag(
                "allow_primitive_interpolation",
                defaults.allow_primitive_interpolation,
            )?,
            downsample_obstacle_heuristic: flag(
                "downsample_obstacle_heuristic",
                defaults.downsample_obstacle_heuristic,
            )?,
        };
        // The global costmap's cells; another size makes the moves again.
        let resolution = number("resolution", 0.25)?;
        return Planner::hybrid(
            spec_hybrid,
            settings,
            &outline,
            inflation.as_ref(),
            resolution,
        );
    }
    let file = spec["lattice_filepath"]
        .as_str()
        .ok_or_else(|| wrong("lattice_filepath: a path is needed".into()))?;
    let text = std::fs::read_to_string(file).map_err(|e| wrong(format!("reading {file}: {e}")))?;
    let lattice = Lattice::parse(&text)?;
    Ok(Planner::new(
        lattice,
        settings,
        &outline,
        inflation.as_ref(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn hybrid_block() -> Value {
        json!({"search": "hybrid", "motion_model_for_search": "REEDS_SHEPP",
               "minimum_turning_radius": 3.5, "resolution": 0.25,
               "footprint": [[1.8, 0.5], [1.8, -0.5], [-0.2, -0.5], [-0.2, 0.5]],
               "footprint_padding": 0.05, "inflation": {"radius": 3.0, "scaling": 1.0},
               "smoother": {"w_data": 0.02}})
    }

    #[test]
    fn a_hybrid_block_makes_a_planner() {
        assert!(planner_from_json(&hybrid_block()).is_ok());
    }

    #[test]
    fn wrong_values_are_refused_by_name() {
        let mut b = hybrid_block();
        b["motion_model_for_search"] = json!("SPIN");
        let e = planner_from_json(&b).err().map(|e| e.to_string());
        assert!(e.is_some_and(|e| e.contains("SPIN")));
        let mut b = hybrid_block();
        b["smoother"]["w_data"] = json!("lots");
        let e = planner_from_json(&b).err().map(|e| e.to_string());
        assert!(e.is_some_and(|e| e.contains("smoother.w_data")));
        let mut b = hybrid_block();
        b["minimum_turning_radius"] = Value::Null;
        assert!(planner_from_json(&b).is_err());
        let lattice = json!({"footprint": [[1.0, 0.5]], "lattice_filepath": "/no/such.json"});
        assert!(planner_from_json(&lattice).is_err());
    }
}
