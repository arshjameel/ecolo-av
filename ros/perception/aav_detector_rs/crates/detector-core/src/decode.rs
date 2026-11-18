//! The model's two raw outputs as boxes in the picture's pixels: `detr.py`'s `decode`.

use crate::{ClassId, Error, Result};

/// A box's corners in the picture's pixels: left, top, right, bottom.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Corners {
    /// Left edge.
    pub x1: f32,
    /// Top edge.
    pub y1: f32,
    /// Right edge.
    pub x2: f32,
    /// Bottom edge.
    pub y2: f32,
}

impl Corners {
    /// Width in pixels.
    pub fn width(&self) -> f32 {
        self.x2 - self.x1
    }

    /// Height in pixels.
    pub fn height(&self) -> f32 {
        self.y2 - self.y1
    }

    /// The centre, x and y.
    pub fn centre(&self) -> (f32, f32) {
        ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)
    }

    /// Intersection over union with `other`: 1 for the same box, 0 for none shared.
    pub fn iou(&self, other: &Corners) -> f32 {
        let w = (self.x2.min(other.x2) - self.x1.max(other.x1)).max(0.0);
        let h = (self.y2.min(other.y2) - self.y1.max(other.y1)).max(0.0);
        let inter = w * h;
        let union = self.width() * self.height() + other.width() * other.height() - inter;
        if union > 0.0 {
            inter / union
        } else {
            0.0
        }
    }
}

/// One thing found: where, how sure, and what.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Detection {
    /// The box, in the picture's pixels.
    pub corners: Corners,
    /// The score, 0 to 1 (the sigmoid of the logit).
    pub score: f32,
    /// The class, by COCO's number for the stock models.
    pub class: ClassId,
}

/// How many boxes to keep: those over `threshold`, of the best `num_select` pairs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DecodeOptions {
    /// The lowest score kept, 0 to 1 (the detector's `confidence`).
    pub threshold: f32,
    /// RF-DETR's `num_select`: the best (query, class) pairs looked at, 300.
    pub num_select: usize,
}

impl Default for DecodeOptions {
    fn default() -> Self {
        Self {
            threshold: 0.5,
            num_select: 300,
        }
    }
}

fn sigmoid(logit: f32) -> f32 {
    1.0 / (1.0 + (-logit.clamp(-88.0, 88.0)).exp())
}

/// Return one picture's detections, best first.
///
/// `boxes` are the model's Q boxes (centre x, centre y, width, height, as 0-1 of the
/// picture), `logits` its Q x `classes` raw scores, both row-major (a batch of one);
/// `width` and `height` are the original picture's, in pixels. As RF-DETR's PostProcess:
/// a sigmoid on every score, the best `num_select` (query, class) pairs of all (one query
/// can be two classes), those over the threshold, their boxes clamped inside the picture.
pub fn decode(
    boxes: &[f32],
    logits: &[f32],
    classes: usize,
    width: usize,
    height: usize,
    options: DecodeOptions,
) -> Result<Vec<Detection>> {
    let queries = boxes.len() / 4;
    if classes == 0 || !boxes.len().is_multiple_of(4) || logits.len() != queries * classes {
        return Err(Error::Shape(format!(
            "boxes must be (Q, 4) and logits (Q, C): got {} and {} floats for C = {classes}",
            boxes.len(),
            logits.len()
        )));
    }
    let mut best: Vec<(usize, f32)> = logits
        .iter()
        .map(|&l| sigmoid(l))
        .enumerate()
        .filter(|&(_, score)| score > options.threshold)
        .collect();
    // Highest first; a stable sort keeps equal scores in query order, as NumPy's does.
    best.sort_by(|a, b| b.1.total_cmp(&a.1));
    best.truncate(options.num_select);
    let (w, h) = (width as f32, height as f32);
    Ok(best
        .into_iter()
        .map(|(i, score)| {
            let (query, class) = (i / classes, i % classes);
            let b = &boxes[4 * query..4 * query + 4];
            let (cx, cy, bw, bh) = (b[0], b[1], b[2], b[3]);
            Detection {
                corners: Corners {
                    x1: ((cx - bw / 2.0) * w).clamp(0.0, w),
                    y1: ((cy - bh / 2.0) * h).clamp(0.0, h),
                    x2: ((cx + bw / 2.0) * w).clamp(0.0, w),
                    y2: ((cy + bh / 2.0) * h).clamp(0.0, h),
                },
                score,
                class: ClassId(u16::try_from(class).unwrap_or(u16::MAX)),
            }
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two queries, three classes; logit 2.0 is a score of 0.88.
    fn outputs() -> (Vec<f32>, Vec<f32>) {
        let boxes = vec![0.5, 0.5, 0.2, 0.4, 0.95, 0.1, 0.2, 0.4];
        let logits = vec![-5.0, 2.0, -5.0, 1.0, -5.0, 3.0];
        (boxes, logits)
    }

    #[test]
    fn best_first_in_pixels_clamped() {
        let (boxes, logits) = outputs();
        let found = decode(&boxes, &logits, 3, 100, 50, DecodeOptions::default()).unwrap();
        assert_eq!(found.len(), 3);
        // Query 1, class 2: the best; its box runs off the right edge and the top.
        assert_eq!(found[0].class, ClassId(2));
        let c = found[0].corners;
        assert!((c.x1 - 85.0).abs() < 1e-4 && c.x2 == 100.0 && c.y1 == 0.0);
        assert!((c.y2 - 15.0).abs() < 1e-4);
        assert!((found[0].score - sigmoid(3.0)).abs() < 1e-7);
        // Query 0, class 1, then query 1 again as class 0: one query, two classes.
        assert_eq!(found[1].class, ClassId(1));
        let want = Corners {
            x1: 40.0,
            y1: 15.0,
            x2: 60.0,
            y2: 35.0,
        };
        assert!(found[1].corners.iou(&want) > 0.9999);
        assert_eq!(found[2].class, ClassId(0));
    }

    #[test]
    fn threshold_and_num_select() {
        let (boxes, logits) = outputs();
        let strict = DecodeOptions {
            threshold: 0.9,
            num_select: 300,
        };
        assert_eq!(
            decode(&boxes, &logits, 3, 100, 50, strict).unwrap().len(),
            1
        );
        let one = DecodeOptions {
            threshold: 0.5,
            num_select: 1,
        };
        let found = decode(&boxes, &logits, 3, 100, 50, one).unwrap();
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].class, ClassId(2));
    }

    #[test]
    fn wrong_shapes_are_errors() {
        let (boxes, logits) = outputs();
        assert!(decode(&boxes, &logits, 4, 100, 50, DecodeOptions::default()).is_err());
        assert!(decode(&boxes[..5], &logits, 3, 100, 50, DecodeOptions::default()).is_err());
        assert!(decode(&boxes, &logits, 0, 100, 50, DecodeOptions::default()).is_err());
    }

    #[test]
    fn iou_of_boxes() {
        let a = Corners {
            x1: 0.0,
            y1: 0.0,
            x2: 2.0,
            y2: 2.0,
        };
        let b = Corners {
            x1: 1.0,
            y1: 0.0,
            x2: 3.0,
            y2: 2.0,
        };
        assert_eq!(a.iou(&a), 1.0);
        assert!((a.iou(&b) - 1.0 / 3.0).abs() < 1e-6);
        assert_eq!(
            a.iou(&Corners {
                x1: 5.0,
                y1: 5.0,
                x2: 6.0,
                y2: 6.0
            }),
            0.0
        );
    }
}
