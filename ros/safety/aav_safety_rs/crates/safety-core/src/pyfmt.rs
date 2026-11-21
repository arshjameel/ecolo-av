//! Numbers written as Python writes them, so the lines for people match the Python's
//! letter for letter: `f'{x:.1f}'` spells NaN `nan`, where Rust spells it `NaN`.

/// `f'{x:.{places}f}'`.
#[must_use]
pub fn fixed(x: f64, places: usize) -> String {
    if x.is_nan() {
        "nan".to_string()
    } else {
        format!("{x:.places$}")
    }
}

/// `f'{x:.1f}'`.
#[must_use]
pub fn f1(x: f64) -> String {
    fixed(x, 1)
}

/// Python's `repr` (and `str`) of a float, for a datum or a setting in a line: written as
/// Python writes it. Python's exponent form (below 1e-4, from 1e16) is not copied: no
/// latitude, longitude or setting reaches it.
#[must_use]
pub fn repr(v: f64) -> String {
    if v.is_nan() {
        "nan".to_string()
    } else if v.is_infinite() {
        if v > 0.0 { "inf" } else { "-inf" }.to_string()
    } else if v == v.trunc() {
        format!("{v:.1}")
    } else {
        format!("{v}")
    }
}

/// Python's `f'{x:g}'`: six significant digits, no trailing zeros (exponent form not
/// copied, as [`repr`]).
#[must_use]
pub fn g(v: f64) -> String {
    if !v.is_finite() {
        return repr(v);
    }
    if v == 0.0 {
        return if v.is_sign_negative() { "-0" } else { "0" }.to_string();
    }
    let digits = 5 - v.abs().log10().floor() as i32;
    let places = usize::try_from(digits.max(0)).unwrap_or(0);
    let s = format!("{v:.places$}");
    if s.contains('.') {
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    } else {
        s
    }
}

/// Python 3.12's `sum()` of floats: Neumaier's compensated sum, so a total matches the
/// Python's to the last bit.
pub fn sum(values: impl IntoIterator<Item = f64>) -> f64 {
    let (mut total, mut c) = (0.0_f64, 0.0_f64);
    for x in values {
        let t = total + x;
        if total.abs() >= x.abs() {
            c += (total - t) + x;
        } else {
            c += (x - t) + total;
        }
        total = t;
    }
    if c != 0.0 && c.is_finite() {
        total + c
    } else {
        total
    }
}

/// Python 3.12's `math.hypot(x, y)`: CPython's own `vector_norm` (exact squares and sums,
/// then one correction step), not the C library's `hypot`. The two differ in the last
/// bit now and then; this one gives the Python's answer to the bit.
#[must_use]
pub fn hypot(x: f64, y: f64) -> f64 {
    let (x, y) = (x.abs(), y.abs());
    let top = [x, y]
        .into_iter()
        .fold(0.0, |top, v| if v > top { v } else { top });
    if top.is_infinite() {
        return top;
    }
    if x.is_nan() || y.is_nan() {
        return f64::NAN;
    }
    if top == 0.0 {
        return top;
    }
    norm(&[x, y], top)
}

fn norm(values: &[f64], top: f64) -> f64 {
    // frexp: top = m * 2^e with m in [0.5, 1).
    let e = exponent(top);
    if e < -1023 {
        let scaled: Vec<f64> = values.iter().map(|v| v / f64::MIN_POSITIVE).collect();
        return f64::MIN_POSITIVE * norm(&scaled, top / f64::MIN_POSITIVE);
    }
    let scale = power_of_two(-e);
    let (mut csum, mut frac1, mut frac2) = (1.0_f64, 0.0_f64, 0.0_f64);
    for &v in values {
        let x = v * scale;
        let (hi, lo) = mul_exact(x, x);
        let (s_hi, s_lo) = fast_sum(csum, hi);
        csum = s_hi;
        frac1 += lo;
        frac2 += s_lo;
    }
    let mut h = (csum - 1.0 + (frac1 + frac2)).sqrt();
    let (hi, lo) = mul_exact(-h, h);
    let (s_hi, s_lo) = fast_sum(csum, hi);
    csum = s_hi;
    frac1 += lo;
    frac2 += s_lo;
    let x = csum - 1.0 + (frac1 + frac2);
    h += x / (2.0 * h);
    h / scale
}

/// frexp's exponent for a normal or subnormal positive number.
fn exponent(v: f64) -> i32 {
    let bits = v.to_bits();
    let biased = ((bits >> 52) & 0x7ff) as i32;
    if biased == 0 {
        // Subnormal: scale up into the normals first.
        exponent(v * 2f64.powi(64)) - 64
    } else {
        biased - 1022
    }
}

/// 2^e exactly, subnormals included (`ldexp(1.0, e)`), for e in -1074..=1023.
fn power_of_two(e: i32) -> f64 {
    if e >= -1022 {
        f64::from_bits(u64::from((e + 1023).unsigned_abs()) << 52)
    } else {
        f64::from_bits(1u64 << (e + 1074).unsigned_abs())
    }
}

fn mul_exact(x: f64, y: f64) -> (f64, f64) {
    let z = x * y;
    (z, x.mul_add(y, -z))
}

fn fast_sum(a: f64, b: f64) -> (f64, f64) {
    let x = a + b;
    let z = x - a;
    (x, b - z)
}

/// `math.sin(x)`: the C library's `sin` alone. An optimised build may join a `sin` and a
/// `cos` of the same angle into one `sincos` call, whose answer can differ from `sin`'s in
/// the last bit (seen on macOS); hiding the angle keeps them apart, as Python's are.
#[must_use]
pub fn sin(x: f64) -> f64 {
    std::hint::black_box(x).sin()
}

/// `math.cos(x)`: the C library's `cos` alone (see [`sin`]).
#[must_use]
pub fn cos(x: f64) -> f64 {
    std::hint::black_box(x).cos()
}

/// Python's `min(a, b)`: `a` unless `b < a` (so a NaN first stays).
#[must_use]
pub fn min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// Python's `max(a, b)`: `a` unless `b > a`.
#[must_use]
pub fn max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn as_python_writes_them() {
        assert_eq!(f1(f64::NAN), "nan");
        assert_eq!(f1(-0.04), "-0.0");
        assert_eq!(f1(0.25), "0.2");
        assert_eq!(f1(f64::INFINITY), "inf");
        assert_eq!(fixed(1.005, 2), "1.00");
    }

    #[test]
    fn compensated() {
        assert_eq!(sum([0.1; 10]), 1.0);
        assert_eq!(sum([1e100, 1.0, -1e100]), 1.0);
    }

    #[test]
    fn min_max_keep_the_first() {
        assert!(min(f64::NAN, 1.0).is_nan());
        assert_eq!(min(1.0, f64::NAN), 1.0);
        assert_eq!(max(2.0, 3.0), 3.0);
    }
}
