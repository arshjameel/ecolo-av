"""Crop and shrink the developer window screenshots for docs/release-0.4-guide.md."""
import sys

from PIL import Image

src, out = sys.argv[1], sys.argv[2]
dw2 = Image.open(f'{src}/dw2.png').convert('RGB')
dw3 = Image.open(f'{src}/dw3.png').convert('RGB')


def save(img, name, width):
    h = round(img.height * width / img.width)
    img.resize((width, h), Image.LANCZOS).save(f'{out}/{name}', quality=85, optimize=True)


save(dw3, 'release-0.4-dev-window.jpg', 1600)
save(dw3.crop((16, 251, 1840, 1277)), 'release-0.4-ravensim.jpg', 1200)
save(dw3.crop((1872, 800, 3200, 1565)), 'release-0.4-map.jpg', 1100)
save(dw2.crop((1856, 99, 3200, 739)), 'release-0.4-things.jpg', 1100)
