//! Numbers written as Python's f-strings write them, so the status lines are the same text.

/// `value` with `places` decimals, as Python's `f'{value:.Nf}'` (`plus`: `:+.Nf`).
///
/// Rust and Python both round the exact binary value to nearest, ties to even, and both
/// keep the sign of -0.0; only the words for infinity and NaN differ, so they are mended
/// here.
pub fn fixed(value: f64, places: usize, plus: bool) -> String {
    let sign = if plus && !value.is_sign_negative() {
        "+"
    } else {
        ""
    };
    if value.is_nan() {
        return format!("{sign}nan");
    }
    if value.is_infinite() {
        return if value > 0.0 {
            format!("{sign}inf")
        } else {
            "-inf".to_string()
        };
    }
    format!("{sign}{value:.places$}")
}

#[cfg(test)]
mod tests {
    use super::fixed;

    #[test]
    fn as_python_writes_them() {
        assert_eq!(fixed(0.1234, 3, true), "+0.123");
        assert_eq!(fixed(-0.1236, 3, true), "-0.124");
        assert_eq!(fixed(-0.0, 1, false), "-0.0");
        assert_eq!(fixed(-0.0, 3, true), "-0.000");
        assert_eq!(fixed(0.0, 3, true), "+0.000");
        assert_eq!(fixed(9999.0, 1, false), "9999.0");
        assert_eq!(fixed(0.125, 2, false), "0.12"); // a tie: to even, as Python
        assert_eq!(fixed(0.375, 2, false), "0.38");
        assert_eq!(fixed(f64::NAN, 2, false), "nan");
        assert_eq!(fixed(f64::INFINITY, 1, true), "+inf");
        assert_eq!(fixed(f64::NEG_INFINITY, 1, true), "-inf");
    }
}
