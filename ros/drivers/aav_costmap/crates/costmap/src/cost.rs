//! Nav2's cost codes (`cost_values.hpp`), and how a cost becomes an `OccupancyGrid` value.

/// Nothing there.
pub const FREE: u8 = 0;
/// The car's inscribed circle, centred here, would touch an obstacle.
pub const INSCRIBED: u8 = 253;
/// An obstacle.
pub const LETHAL: u8 = 254;
/// Nobody has seen this cell.
pub const NO_INFORMATION: u8 = 255;

/// The `OccupancyGrid` value for a cost, as Nav2's `Costmap2DPublisher` writes it:
/// 0 free, 99 inscribed, 100 lethal, -1 unknown, 1..98 in between.
#[must_use]
pub fn occupancy(cost: u8) -> i8 {
    match cost {
        FREE => 0,
        INSCRIBED => 99,
        LETHAL => 100,
        NO_INFORMATION => -1,
        // 1 + 97 (i - 1) / 251, in integers: 1 -> 1, 252 -> 98.
        c => (1 + (97 * (u32::from(c) - 1)) / 251) as i8,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_publisher_table() {
        assert_eq!(occupancy(FREE), 0);
        assert_eq!(occupancy(1), 1);
        assert_eq!(occupancy(252), 98);
        assert_eq!(occupancy(INSCRIBED), 99);
        assert_eq!(occupancy(LETHAL), 100);
        assert_eq!(occupancy(NO_INFORMATION), -1);
        // Never falls as the cost rises.
        assert!((1..=254u8).all(|c| occupancy(c) >= occupancy(c - 1)));
    }
}
