//! A readable scale: the bar in the panel's corner, and the grid's lines.

/// The scale bar: a round length, and how long it is on the screen.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScaleBar {
    pub metres: f64,
    pub pixels: f64,
}

impl ScaleBar {
    /// The longest round length (1, 2 or 5 times a power of ten) no longer than
    /// `most_pixels` at `pixels_per_metre`; `None` for a scale that isn't a number.
    #[must_use]
    pub fn fit(pixels_per_metre: f64, most_pixels: f64) -> Option<Self> {
        if !(pixels_per_metre.is_finite() && pixels_per_metre > 0.0 && most_pixels > 0.0) {
            return None;
        }
        let metres = round_down(most_pixels / pixels_per_metre);
        Some(Self {
            metres,
            pixels: metres * pixels_per_metre,
        })
    }

    /// "5 m", "50 cm", "2 km".
    #[must_use]
    pub fn label(&self) -> String {
        length_label(self.metres)
    }
}

/// A length in the unit that reads best.
#[must_use]
pub fn length_label(metres: f64) -> String {
    if metres >= 1000.0 {
        format!("{} km", trim(metres / 1000.0))
    } else if metres >= 1.0 {
        format!("{} m", trim(metres))
    } else {
        format!("{} cm", trim(metres * 100.0))
    }
}

fn trim(value: f64) -> String {
    let text = format!("{value:.1}");
    text.strip_suffix(".0").map_or(text.clone(), str::to_owned)
}

/// The largest of 1, 2, 5 times a power of ten that is no more than `length`.
fn round_down(length: f64) -> f64 {
    let power = 10f64.powf(length.log10().floor());
    [5.0, 2.0, 1.0]
        .into_iter()
        .map(|k| k * power)
        .find(|step| *step <= length * (1.0 + 1e-9))
        .unwrap_or(power)
}

/// The grid's spacing, metres: the smallest round length whose lines are at least
/// `least_pixels` apart; `None` for a scale that isn't a number.
#[must_use]
pub fn grid_step(pixels_per_metre: f64, least_pixels: f64) -> Option<f64> {
    if !(pixels_per_metre.is_finite() && pixels_per_metre > 0.0 && least_pixels > 0.0) {
        return None;
    }
    let least = least_pixels / pixels_per_metre;
    let power = 10f64.powf(least.log10().floor());
    [1.0, 2.0, 5.0, 10.0]
        .into_iter()
        .map(|k| k * power)
        .find(|step| *step >= least * (1.0 - 1e-9))
}

/// Where the grid's lines are between `from` and `to` (metres, one axis), every `step`.
pub fn grid_lines(from: f64, to: f64, step: f64) -> impl Iterator<Item = f64> {
    let ok = step > 0.0 && from.is_finite() && to.is_finite() && to >= from;
    let first = if ok { (from / step).ceil() as i64 } else { 1 };
    let last = if ok { (to / step).floor() as i64 } else { 0 };
    (first..=last).map(move |i| i as f64 * step)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_bar_is_a_round_length_that_fits() {
        // 12 pixels a metre, at most 120 pixels: 10 m.
        let bar = ScaleBar::fit(12.0, 120.0).expect("a real scale");
        assert_eq!(bar.metres, 10.0);
        assert_eq!(bar.pixels, 120.0);
        assert_eq!(bar.label(), "10 m");
        // Zoomed far in: 300 pixels a metre.
        let near = ScaleBar::fit(300.0, 120.0).expect("a real scale");
        assert!((near.metres - 0.2).abs() < 1e-12);
        assert_eq!(near.label(), "20 cm");
        // A whole campus: 0.05 pixels a metre.
        let far = ScaleBar::fit(0.05, 120.0).expect("a real scale");
        assert_eq!(far.metres, 2000.0);
        assert_eq!(far.label(), "2 km");
    }

    #[test]
    fn a_scale_that_is_not_a_number_has_no_bar() {
        assert_eq!(ScaleBar::fit(0.0, 120.0), None);
        assert_eq!(ScaleBar::fit(f64::NAN, 120.0), None);
        assert_eq!(grid_step(-1.0, 40.0), None);
    }

    #[test]
    fn the_grid_keeps_its_lines_apart() {
        assert_eq!(grid_step(40.0, 40.0), Some(1.0));
        assert_eq!(grid_step(12.0, 40.0), Some(5.0));
        assert_eq!(grid_step(1.0, 40.0), Some(50.0));
    }

    #[test]
    fn the_grids_lines_are_on_round_metres() {
        let lines: Vec<f64> = grid_lines(-7.0, 12.0, 5.0).collect();
        assert_eq!(lines, [-5.0, 0.0, 5.0, 10.0]);
        assert_eq!(grid_lines(3.0, 1.0, 5.0).count(), 0);
        assert_eq!(grid_lines(0.0, 10.0, 0.0).count(), 0);
    }
}
