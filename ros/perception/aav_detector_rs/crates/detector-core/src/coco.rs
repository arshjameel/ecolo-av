//! COCO's class names, by COCO's own numbers: RF-DETR's stock models answer in these.

use std::fmt;

/// A class number, as the model's logits number them (COCO's: 1 = person, 13 = stop sign).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ClassId(pub u16);

impl ClassId {
    /// The class's COCO name ("stop sign"), as rfdetr's `COCO_CLASSES` has it; `None` for
    /// a number COCO leaves unused.
    pub fn name(self) -> Option<&'static str> {
        NAMES
            .binary_search_by_key(&self.0, |&(id, _)| id)
            .ok()
            .map(|i| NAMES[i].1)
    }
}

impl fmt::Display for ClassId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.name() {
            Some(name) => f.write_str(name),
            None => write!(f, "class {}", self.0),
        }
    }
}

/// A label as the detector compares them: "Stop_Sign" (the 2024-25 model's) and "stop
/// sign" (the stock model's) are the same. As aav_detector's `boxes.normal_label`.
pub fn normal_label(label: &str) -> String {
    label
        .replace(['_', '-'], " ")
        .to_lowercase()
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
}

/// The labels parameter, "stop sign, person": the labels to keep; `None` (keep them all)
/// when empty. As aav_detector's `boxes.parse_labels`.
pub fn parse_labels(text: &str) -> Option<Vec<String>> {
    let labels: Vec<String> = text
        .split(',')
        .filter(|s| !s.trim().is_empty())
        .map(normal_label)
        .collect();
    (!labels.is_empty()).then_some(labels)
}

/// rfdetr 1.11's `rfdetr.assets.coco_classes.COCO_CLASSES` (Apache-2.0), sorted by number.
const NAMES: [(u16, &str); 80] = [
    (1, "person"),
    (2, "bicycle"),
    (3, "car"),
    (4, "motorcycle"),
    (5, "airplane"),
    (6, "bus"),
    (7, "train"),
    (8, "truck"),
    (9, "boat"),
    (10, "traffic light"),
    (11, "fire hydrant"),
    (13, "stop sign"),
    (14, "parking meter"),
    (15, "bench"),
    (16, "bird"),
    (17, "cat"),
    (18, "dog"),
    (19, "horse"),
    (20, "sheep"),
    (21, "cow"),
    (22, "elephant"),
    (23, "bear"),
    (24, "zebra"),
    (25, "giraffe"),
    (27, "backpack"),
    (28, "umbrella"),
    (31, "handbag"),
    (32, "tie"),
    (33, "suitcase"),
    (34, "frisbee"),
    (35, "skis"),
    (36, "snowboard"),
    (37, "sports ball"),
    (38, "kite"),
    (39, "baseball bat"),
    (40, "baseball glove"),
    (41, "skateboard"),
    (42, "surfboard"),
    (43, "tennis racket"),
    (44, "bottle"),
    (46, "wine glass"),
    (47, "cup"),
    (48, "fork"),
    (49, "knife"),
    (50, "spoon"),
    (51, "bowl"),
    (52, "banana"),
    (53, "apple"),
    (54, "sandwich"),
    (55, "orange"),
    (56, "broccoli"),
    (57, "carrot"),
    (58, "hot dog"),
    (59, "pizza"),
    (60, "donut"),
    (61, "cake"),
    (62, "chair"),
    (63, "couch"),
    (64, "potted plant"),
    (65, "bed"),
    (67, "dining table"),
    (70, "toilet"),
    (72, "tv"),
    (73, "laptop"),
    (74, "mouse"),
    (75, "remote"),
    (76, "keyboard"),
    (77, "cell phone"),
    (78, "microwave"),
    (79, "oven"),
    (80, "toaster"),
    (81, "sink"),
    (82, "refrigerator"),
    (84, "book"),
    (85, "clock"),
    (86, "vase"),
    (87, "scissors"),
    (88, "teddy bear"),
    (89, "hair drier"),
    (90, "toothbrush"),
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_by_coco_number() {
        assert_eq!(ClassId(1).name(), Some("person"));
        assert_eq!(ClassId(13).name(), Some("stop sign"));
        assert_eq!(ClassId(12).name(), None); // COCO skips 12
        assert_eq!(ClassId(12).to_string(), "class 12");
        assert!(NAMES.windows(2).all(|w| w[0].0 < w[1].0));
    }

    #[test]
    fn labels_as_the_python_reads_them() {
        assert_eq!(normal_label(" Stop_Sign "), "stop sign");
        assert_eq!(normal_label("traffic-light"), "traffic light");
        assert_eq!(
            parse_labels("stop sign, Person ,"),
            Some(vec!["stop sign".to_owned(), "person".to_owned()])
        );
        assert_eq!(parse_labels(" , "), None);
    }
}
