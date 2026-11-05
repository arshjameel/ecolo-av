#!/usr/bin/env python3
"""
Make the team's arcade logo, in two variants, for the HUD's `initial_d` theme.

An homage to the look of Initial D's title, not a copy of it: our own words, our own
layout, open fonts. A small bracketed katakana line, カールトン (Carleton), above a heavy
slanted word with a big red italic V, and speed lines behind:

  katakana-led   〔カールトン〕 / エーエー V   (エーエー: "A A", read in katakana)
  latin-led      〔カールトン〕 / AA V         (the HUD shows this one)

Both read "AAV". Black background, white words, Carleton's red (#e91c24, the "Red"
swatch in Carleton's Logo Guide 2023's RGB logo files; check it against the brand
guide). No Carleton, Ravens, Sega or Kodansha artwork is used.

Writes:
  docs/img/logo-katakana.svg, .png    docs/img/logo-latin.svg, .png
  ros/hud/aav_hud/assets/hud-logo.png  the latin one, as the HUD builds it in

The SVGs are paths (no font needed to show them). The fonts, both SIL Open Font
Licence 1.1, are downloaded once into vendor/fonts/ (not in git) from google/fonts at a
fixed commit, with their OFL.txt:
  Dela Gothic One (the katakana)       Kanit Black Italic (the Latin)
The OFL lets anyone use pictures made with a font; the fonts themselves aren't committed.

usage: pixi run python3 tools/make_logo.py   (needs matplotlib, not ROS)
"""

from pathlib import Path
import sys
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
FONTS = ROOT / 'vendor' / 'fonts'
IMG = ROOT / 'docs' / 'img'
HUD_LOGO = ROOT / 'ros' / 'hud' / 'aav_hud' / 'assets' / 'hud-logo.png'

# google/fonts on 1 Nov 2026: the files below, and each family's licence.
GOOGLE_FONTS = ('https://raw.githubusercontent.com/google/fonts/'
                '6c1dbe27dcf7d817d4b62cec033ba77fa8e2a5ec')
FONT_FILES = {
    'DelaGothicOne-Regular.ttf': 'ofl/delagothicone/DelaGothicOne-Regular.ttf',
    'DelaGothicOne-OFL.txt': 'ofl/delagothicone/OFL.txt',
    'Kanit-BlackItalic.ttf': 'ofl/kanit/Kanit-BlackItalic.ttf',
    'Kanit-OFL.txt': 'ofl/kanit/OFL.txt',
}

BLACK = '#000000'
WHITE = '#ffffff'
RED = '#e91c24'   # hud_core::theme::CARLETON_RED
SLANT_DEG = 14    # the heavy word's lean (Kanit's italic leans already)


def fetch_fonts():
    """Download the fonts and their licences into vendor/fonts/, once."""
    FONTS.mkdir(parents=True, exist_ok=True)
    for name, path in FONT_FILES.items():
        target = FONTS / name
        if not target.exists():
            print(f'downloading {path}')
            urllib.request.urlretrieve(f'{GOOGLE_FONTS}/{path}', target)
    return FONTS / 'DelaGothicOne-Regular.ttf', FONTS / 'Kanit-BlackItalic.ttf'


def text_path(text, font, size, slant=0.0):
    """Return `text` as a matplotlib path, its bottom left at (0, 0), leaning `slant`."""
    from matplotlib.font_manager import FontProperties
    from matplotlib.textpath import TextPath
    from matplotlib.transforms import Affine2D

    path = TextPath((0, 0), text, size=size, prop=FontProperties(fname=str(font)))
    path = path.transformed(Affine2D().skew_deg(slant, 0))
    box = path.get_extents()
    return path.transformed(Affine2D().translate(-box.x0, -box.y0))


def logo(kana_font, latin_font, heavy, heavy_font, heavy_slant):
    """Return the logo's pieces: (path, x, y, colour) each, and its size (w, h)."""
    word = text_path(heavy, heavy_font, 1.0, heavy_slant)
    letter = text_path('V', latin_font, 1.75)
    word_w = word.get_extents().width
    word_h = word.get_extents().height
    letter_box = letter.get_extents()
    x0 = 0.75                        # room on the left for the speed lines
    word_x, word_y = x0, 0.18
    letter_x = word_x + word_w - 0.06
    # The small line fits over the word, clear of the big letter.
    room = letter_x - (word_x + 0.10) - 0.12
    small = text_path('〔カールトン〕', kana_font, 0.30, SLANT_DEG * 0.6)
    if small.get_extents().width > room:
        size = 0.30 * room / small.get_extents().width
        small = text_path('〔カールトン〕', kana_font, size, SLANT_DEG * 0.6)
    pieces = [
        (small, word_x + 0.10, word_y + word_h + 0.12, WHITE),
        (word, word_x, word_y, WHITE),
        (letter, letter_x, 0.0, RED),
    ]
    width = letter_x + letter_box.width + 0.15
    height = max(word_y + word_h + 0.12 + small.get_extents().height,
                 letter_box.height) + 0.12
    return pieces, (width, height), (word_x, word_y, word_h)


def speed_lines(word_x, word_y, word_h):
    """Return the speed lines: (x0, x1, y, width, colour), streaming back from the word."""
    lines = []
    rows = [0.10, 0.24, 0.40, 0.52, 0.66, 0.80, 0.92]
    lengths = [0.55, 0.80, 0.45, 0.85, 0.60, 0.75, 0.40]
    for k, (row, length) in enumerate(zip(rows, lengths)):
        y = word_y + row * word_h
        start = word_x + 0.25 - length + 0.12 * (k % 2)
        colour = RED if k in (1, 4) else WHITE
        lines.append((start, word_x + 0.25, y, 1.6 if k % 3 else 2.6, colour))
    return lines


def draw(pieces, size, word, svg, png, png_width):
    """Draw the logo and save it as an SVG (paths) and a PNG `png_width` pixels wide."""
    import matplotlib
    matplotlib.use('Agg')
    matplotlib.rcParams['svg.fonttype'] = 'path'
    import matplotlib.pyplot as plt
    from matplotlib.patches import PathPatch
    from matplotlib.transforms import Affine2D

    width, height = size
    fig = plt.figure(figsize=(width, height), dpi=100)
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.set_axis_off()
    fig.patch.set_facecolor(BLACK)
    for x0, x1, y, lw, colour in speed_lines(*word):
        ax.plot([x0, x1], [y, y], color=colour, linewidth=lw, solid_capstyle='butt',
                zorder=1)
    for path, x, y, colour in pieces:
        placed = path.transformed(Affine2D().translate(x, y))
        # A black edge round each word: it cuts the speed lines where they meet.
        ax.add_patch(PathPatch(placed, facecolor=colour, edgecolor=BLACK,
                               linewidth=2.0, zorder=2))
    for out in svg:
        fig.savefig(out, facecolor=BLACK, metadata={'Date': None})
    for out in png:
        fig.savefig(out, facecolor=BLACK, dpi=png_width / width)
    plt.close(fig)


def main():
    kana, latin = fetch_fonts()
    IMG.mkdir(parents=True, exist_ok=True)
    HUD_LOGO.parent.mkdir(parents=True, exist_ok=True)
    variants = {
        'katakana': ('エーエー', kana, SLANT_DEG),
        'latin': ('AA', latin, 0.0),
    }
    for name, (heavy, font, slant) in variants.items():
        pieces, size, word = logo(kana, latin, heavy, font, slant)
        pngs = [IMG / f'logo-{name}.png']
        if name == 'latin':
            pngs.append(HUD_LOGO)
        draw(pieces, size, word, [IMG / f'logo-{name}.svg'], pngs, 720)
        for out in [IMG / f'logo-{name}.svg', *pngs]:
            print(f'wrote {out.relative_to(ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
