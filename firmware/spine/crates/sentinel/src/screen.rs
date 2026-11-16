//! What the 16x2 LCD shows: two lines of 16 characters.

use core::fmt;

/// Characters on a line.
pub const WIDTH: usize = 16;

/// One line: always 16 ASCII characters, padded with spaces. Writing past the end is
/// cut off, never an error, so a long number can't break a page.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Line {
    chars: [u8; WIDTH],
    len: usize,
}

impl Line {
    #[must_use]
    pub const fn blank() -> Self {
        Line {
            chars: [b' '; WIDTH],
            len: 0,
        }
    }

    /// A line from formatted text: `Line::from_fmt(format_args!("SPD:{:.2}", v))`.
    #[must_use]
    pub fn from_fmt(args: fmt::Arguments<'_>) -> Self {
        let mut line = Line::blank();
        // Writing to a Line never fails: the extra is dropped.
        let _ = fmt::write(&mut line, args);
        line
    }

    /// The 16 characters, for the display.
    #[must_use]
    pub fn bytes(&self) -> &[u8; WIDTH] {
        &self.chars
    }

    /// The text, with the padding.
    #[must_use]
    pub fn as_str(&self) -> &str {
        // Only ASCII ever goes in (`write_str` replaces the rest).
        core::str::from_utf8(&self.chars).unwrap_or("")
    }

    /// Put `c` in the last column: the page's pin mark.
    #[must_use]
    pub fn with_last(mut self, c: u8) -> Self {
        self.chars[WIDTH - 1] = c;
        self
    }
}

impl fmt::Write for Line {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.chars() {
            if self.len == WIDTH {
                break;
            }
            // The LCD's font has ASCII; anything else shows as '?'.
            self.chars[self.len] = if c.is_ascii() { c as u8 } else { b'?' };
            self.len += 1;
        }
        Ok(())
    }
}

impl fmt::Debug for Line {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self.as_str())
    }
}

/// The two lines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Screen {
    pub top: Line,
    pub bottom: Line,
}

/// `lcd_line!("MODE:{}", mode)`: one 16-character line, formatted.
#[macro_export]
macro_rules! lcd_line {
    ($($arg:tt)*) => {
        $crate::screen::Line::from_fmt(format_args!($($arg)*))
    };
}

#[cfg(test)]
mod tests {

    #[test]
    fn lines_are_padded() {
        assert_eq!(lcd_line!("OK").as_str(), "OK              ");
    }

    #[test]
    fn long_lines_are_cut() {
        assert_eq!(
            lcd_line!("{}", "0123456789ABCDEFGHIJ").as_str(),
            "0123456789ABCDEF"
        );
    }

    #[test]
    fn non_ascii_shows_as_a_question_mark() {
        assert_eq!(lcd_line!("45°N").as_str(), "45?N            ");
    }

    #[test]
    fn the_pin_mark_goes_last() {
        assert_eq!(
            lcd_line!("PAGE").with_last(b'*').as_str(),
            "PAGE           *"
        );
    }
}
