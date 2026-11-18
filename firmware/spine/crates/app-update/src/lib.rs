//! The checks around a firmware update over WiFi: the ESP32's `wifi-update` feature, the
//! Rust twin of the 2025-26 C++'s `/update` page, which took any upload from anyone on the
//! WiFi. Here an upload is written only when all of these hold, and the board restarts
//! into it only after it was written whole and read back the same.
//!
//! | Check | Here | When |
//! |---|---|---|
//! | the update password from the untracked secrets file, and a lockout after a wrong one | [`Guard`] | before a byte is read |
//! | the car parked: nobody driving, not armed, no throttle, the brake on | [`may_update`] | before, at every chunk, and before the restart |
//! | a valid ESP32 app image for this program, small enough for the slot | [`ImageCheck`] | as the bytes come, before anything is written, and at the end |
//! | the slot holds what came | [`Crc32`] of the bytes, against a read of the slot | after the write |
//!
//! Plain `no_std` Rust with no dependencies: tested on a laptop. The board does the flash
//! writes (`esp-bootloader-esp-idf`'s OTA slots) and the restart.
#![no_std]
// The Rust API Guidelines, kept (TODO 176): every public item documented, its errors
// too, and a pure function's value never dropped.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

mod gate;
mod image;

pub use gate::{
    may_update, CarState, Guard, NotNow, PasswordError, BRAKED_AT_LEAST, LOCKOUT_MS,
    MIN_PASSWORD_LEN,
};
pub use image::{
    Crc32, Expect, ImageCheck, ImageError, ImageInfo, APP_DESC_MAGIC, CHIP_ESP32, HASH_LEN,
    HEADER_LEN, IMAGE_MAGIC, MAX_SEGMENTS,
};
