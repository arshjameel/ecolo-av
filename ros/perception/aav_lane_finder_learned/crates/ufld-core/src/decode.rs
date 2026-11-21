//! The model's answers into each line's points in the picture (`tools/lane_data.py`'s
//! `decode`).

use lane_core::Paint;

use crate::{Error, Meta, Result};

/// One point of a line: where it crosses a row anchor (px, the picture the model was made
/// for), and how sure the model is that the line is there (0 to 1).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Anchor {
    pub u: f64,
    pub v: f64,
    pub p: f64,
}

/// Each line's points, in the model's order (`meta.lanes`).
///
/// `loc`: cells x rows x lanes logits (which column); `exist`: 2 x rows x lanes logits (is the
/// line in that row). A row counts when its chance is above `threshold`; its column is the
/// softmax's mean over the best cell and the one either side (UFLD v2's local expectation).
pub fn decode(loc: &[f32], exist: &[f32], meta: &Meta, threshold: f64) -> Result<Vec<Vec<Anchor>>> {
    let (cells, rows, lanes) = (meta.cells, meta.rows.len(), meta.lanes.len());
    let (n_loc, n_exist) = meta.sizes();
    if loc.len() != n_loc || exist.len() != n_exist {
        return Err(Error::Shape(format!(
            "answers of {} and {} numbers, not {n_loc} and {n_exist}",
            loc.len(),
            exist.len()
        )));
    }
    let at = |c: usize, r: usize, k: usize| f64::from(loc[(c * rows + r) * lanes + k]);
    let step = meta.width as f64 / cells as f64;
    Ok((0..lanes)
        .map(|k| {
            (0..rows)
                .filter_map(|r| {
                    let off = f64::from(exist[r * lanes + k]);
                    let on = f64::from(exist[(rows + r) * lanes + k]);
                    let p = 1.0 / (1.0 + (off - on).exp());
                    if p <= threshold {
                        return None;
                    }
                    let best = (0..cells).max_by(|&a, &b| at(a, r, k).total_cmp(&at(b, r, k)))?;
                    let (lo, hi) = (best.saturating_sub(1), (best + 2).min(cells));
                    let top = at(best, r, k);
                    let (sum, weighted) = (lo..hi).fold((0.0, 0.0), |(s, ws), c| {
                        let w = (at(c, r, k) - top).exp();
                        (s + w, ws + w * c as f64)
                    });
                    Some(Anchor {
                        u: (weighted / sum + 0.5) * step,
                        v: meta.rows[r],
                        p,
                    })
                })
                .collect()
        })
        .collect())
}

/// Each line's paint, in the model's order, from its colour head: 2 x lanes logits, white
/// (`tools/lane_data.py`'s 0) then yellow (1). The head answers for every line; a line not
/// found is left Unknown by [`crate::Learned::lane`].
pub fn paints(colour: &[f32], meta: &Meta) -> Result<Vec<Paint>> {
    let lanes = meta.lanes.len();
    if colour.len() != 2 * lanes {
        return Err(Error::Shape(format!(
            "a colour answer of {} numbers, not 2 x {lanes}",
            colour.len()
        )));
    }
    let (white, yellow) = colour.split_at(lanes);
    Ok(white
        .iter()
        .zip(yellow)
        .map(|(w, y)| if y > w { Paint::Yellow } else { Paint::White })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::meta::example;

    #[test]
    fn reads_the_column_and_skips_rows_without_the_line() {
        let mut meta = example();
        meta.rows = vec![200.0, 220.0, 240.0];
        let (cells, rows, lanes) = (meta.cells, 3, 4);
        let mut loc = vec![-10.0f32; cells * rows * lanes];
        let mut exist = vec![0.0f32; 2 * rows * lanes];
        let idx = |c: usize, r: usize, k: usize| (c * rows + r) * lanes + k;
        for r in 0..rows {
            exist[(rows + r) * lanes + 1] = 5.0; // line 1 in every row
            loc[idx(40, r, 1)] = 10.0;
        }
        loc[idx(41, 1, 1)] = 10.0; // row 1: halfway between 40 and 41
        exist[(rows + 2) * lanes + 2] = 0.0; // line 2: a coin toss, so no
        let lines = decode(&loc, &exist, &meta, 0.5).unwrap();
        assert!(lines[0].is_empty() && lines[2].is_empty() && lines[3].is_empty());
        let step = 6.4;
        assert!((lines[1][0].u - 40.5 * step).abs() < 1e-6);
        assert!((lines[1][1].u - 41.0 * step).abs() < 1e-6);
        assert_eq!(lines[1][2].v, 240.0);
        assert!(lines[1][0].p > 0.99);
        assert!(decode(&loc[1..], &exist, &meta, 0.5).is_err());
    }

    #[test]
    fn the_colour_head_says_white_or_yellow() {
        let meta = example();
        // white logits for the 4 lines, then yellow's: line 1 yellow, the rest white.
        let colour = [2.0, -1.0, 3.0, 0.5, -2.0, 4.0, 1.0, 0.0];
        assert_eq!(
            paints(&colour, &meta).unwrap(),
            [Paint::White, Paint::Yellow, Paint::White, Paint::White]
        );
        assert!(paints(&colour[1..], &meta).is_err());
    }
}
