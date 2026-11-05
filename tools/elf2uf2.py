#!/usr/bin/env python3
"""
Turn a program for the Pico 2 (an RP2350 ELF file) into a UF2 file, to drag onto it.

Hold the Pico 2's BOOTSEL button while plugging it in: it shows up as a drive called
RP2350. Copy the .uf2 file onto that drive, and the Pico 2 restarts into the new program.
`pixi run pico spine build` builds the spine's program and runs this on it.

usage: tools/elf2uf2.py <program.elf> [<program.uf2>]

Standard library only. The rules come from the RP2350 datasheet:
- 5.5.2 and 5.5.3: 256-byte blocks at 256-byte flash addresses, with the family ID
  rp2350_arm_s (0xe48bff59, Table 469), for an image the boot ROM boots;
- 5.9.5: an IMAGE_DEF block in the first 4 kB of the image, or the boot ROM won't run it.
  This checks there is one, for a Secure Arm executable (Tables 487, 492, 493 and 503).
"""

from pathlib import Path
import struct
import sys

FLASH_START = 0x1000_0000
FLASH_END = 0x1200_0000              # 5.5.2: flash images go between these
IMAGE_DEF_WITHIN = 4096              # 5.9.5: the first block is in the first 4 kB
PAGE = 256                           # 5.5.2: every block carries 256 bytes

UF2_MAGIC_START0 = 0x0A32_4655       # "UF2\n" (the UF2 format, github.com/microsoft/uf2)
UF2_MAGIC_START1 = 0x9E5D_5157
UF2_MAGIC_END = 0x0AB1_6F30
UF2_FLAG_FAMILY_ID = 0x0000_2000
UF2_DATA = 476                       # the payload space in each 512-byte block
RP2350_ARM_S = 0xE48B_FF59           # Table 469

BLOCK_START = 0xFFFF_DED3            # PICOBIN_BLOCK_MARKER_START (Table 487)
BLOCK_END = 0xAB12_3579              # PICOBIN_BLOCK_MARKER_END
ITEM_IMAGE_TYPE = 0x42               # PICOBIN_BLOCK_ITEM_1BS_IMAGE_TYPE (Table 492)
ITEM_LAST = 0x7F                     # PICOBIN_BLOCK_ITEM_2BS_LAST, less its size flag
BLOCK_MAX = 640                      # 5.9.1: the boot ROM ignores bigger blocks,
IMAGE_DEF_MAX = 384                  # and bigger IMAGE_DEFs
LOOP_MAX = 16                        # more blocks than this in a loop: not an image of ours

PT_LOAD = 1
EM_ARM = 40


class Uf2Error(Exception):
    """The ELF file can't become a UF2 the RP2350 would boot."""


def load_segments(elf):
    """Return the (load address, bytes) of each loadable part of a 32-bit Arm ELF file."""
    if elf[:4] != b'\x7fELF' or elf[4:6] != b'\x01\x01':
        raise Uf2Error('not a 32-bit little-endian ELF file')
    try:
        machine, = struct.unpack_from('<H', elf, 18)
        phoff, = struct.unpack_from('<I', elf, 28)
        phentsize, phnum = struct.unpack_from('<HH', elf, 42)
        headers = [struct.unpack_from('<5I', elf, phoff + k * phentsize) for k in range(phnum)]
    except struct.error:
        raise Uf2Error('the ELF file is cut short') from None
    if machine != EM_ARM:
        raise Uf2Error(f'built for machine {machine}, not Arm ({EM_ARM})')
    loads = [(offset, paddr, filesz) for p_type, offset, _vaddr, paddr, filesz in headers
             if p_type == PT_LOAD and filesz]
    if not loads:
        raise Uf2Error('nothing to load')
    if any(offset + filesz > len(elf) for offset, _, filesz in loads):
        raise Uf2Error('the ELF file is cut short')
    # Initialised data runs from RAM but loads from flash: its physical address is the one.
    return [(paddr, elf[offset:offset + filesz]) for offset, paddr, filesz in loads]


def flash_pages(segments):
    """Lay the segments into 256-byte pages of flash, keyed by address; gaps stay 0."""
    pages = {}
    for address, data in segments:
        if not (FLASH_START <= address and address + len(data) <= FLASH_END):
            raise Uf2Error(f'{len(data)} bytes at {address:#010x} are not in flash')
        for i, byte in enumerate(data):
            at = address + i
            page = pages.setdefault(at - at % PAGE, bytearray(PAGE))
            page[at % PAGE] = byte
    return pages


def word(pages, address):
    """Return the flash word at `address`; erased flash (no page) reads as all ones."""
    page = pages.get(address - address % PAGE)
    if page is None:
        return 0xFFFF_FFFF
    return struct.unpack_from('<I', page, address % PAGE)[0]


def read_block(pages, start):
    """
    Return the block at `start` as (items, link, size in bytes), or None if it isn't one.

    A block (Table 487): the start marker, the items, a LAST item holding the other items'
    size in words, a link (bytes from this block to the next in the loop), the end marker.
    Each item is (type, size in words, its first word).
    """
    if word(pages, start) != BLOCK_START:
        return None
    items, at = [], start + 4
    while at - start < BLOCK_MAX:
        header = word(pages, at)
        kind, two_byte_size = header & 0x7F, header >> 7 & 1
        size = header >> 8 & (0xFFFF if two_byte_size else 0xFF)
        if kind == ITEM_LAST and two_byte_size:
            if size != sum(n for _, n, _ in items) or word(pages, at + 8) != BLOCK_END:
                return None
            link, = struct.unpack('<i', struct.pack('<I', word(pages, at + 4)))
            return items, link, at + 12 - start
        if size == 0:
            return None
        items.append((kind, size, header))
        at += 4 * size
    return None


def image_defs(pages):
    """
    Return the (address, IMAGE_TYPE flags) of each IMAGE_DEF in the image's block loop.

    The first block is within the first 4 kB, and the links lead back to it (5.9.1, 5.9.5).
    A block is an IMAGE_DEF when its first item is an IMAGE_TYPE (5.9.3.1); a later one in
    the loop takes over from an earlier one (the examples in 5.10).
    """
    for first in range(FLASH_START, FLASH_START + IMAGE_DEF_WITHIN, 4):
        block = read_block(pages, first)
        if block is not None and first + block[2] > FLASH_START + IMAGE_DEF_WITHIN:
            continue                 # it runs past the first 4 kB
        found, at = [], first
        for _ in range(LOOP_MAX):
            if block is None:
                break                # not a closed loop: look further on
            items, link, size = block
            if items and items[0][0] == ITEM_IMAGE_TYPE and size <= IMAGE_DEF_MAX:
                found.append((at, items[0][2] >> 16))
            at += link
            if at == first:
                return found
            block = read_block(pages, at)
    return []


def explain(flags):
    """Return why an IMAGE_DEF with these flags won't boot as ours, or None if it will."""
    checks = [                       # Table 493
        (flags & 0xF, 1, 'not an executable'),
        (flags >> 4 & 3, 2, 'not a Secure image (rp2350_arm_s)'),
        (flags >> 8 & 7, 0, 'not for the Arm cores'),
        (flags >> 12 & 7, 1, 'not for the RP2350'),
    ]
    return next((why for got, want, why in checks if got != want), None)


def uf2(pages):
    """Return the UF2 file: one 512-byte block for each page, in address order."""
    out = bytearray()
    addresses = sorted(pages)
    for number, address in enumerate(addresses):
        out += struct.pack('<8I', UF2_MAGIC_START0, UF2_MAGIC_START1, UF2_FLAG_FAMILY_ID,
                           address, PAGE, number, len(addresses), RP2350_ARM_S)
        out += pages[address] + bytes(UF2_DATA - PAGE)
        out += struct.pack('<I', UF2_MAGIC_END)
    return bytes(out)


def convert(elf):
    """Return the UF2 for an ELF file, and the (address, flags) of the IMAGE_DEF it boots."""
    pages = flash_pages(load_segments(elf))
    found = image_defs(pages)
    if not found:
        raise Uf2Error('no IMAGE_DEF block in the first 4 kB: the RP2350 would not boot it'
                       ' (datasheet 5.9.5)')
    for at, flags in found:
        why = explain(flags)
        if why:
            raise Uf2Error(f'the IMAGE_DEF at {at:#010x} (flags {flags:#06x}) is {why}')
    return uf2(pages), found[-1]


def main(argv):
    """Convert the file named in argv; say what was written."""
    if len(argv) not in (2, 3):
        print(__doc__.split('\n\n')[2], file=sys.stderr)
        return 2
    elf_path = Path(argv[1])
    out_path = Path(argv[2]) if len(argv) == 3 else elf_path.with_suffix('.uf2')
    try:
        data, (at, flags) = convert(elf_path.read_bytes())
        out_path.write_bytes(data)
    except (OSError, Uf2Error) as e:
        print(f'elf2uf2: {elf_path}: {e}', file=sys.stderr)
        return 1
    blocks = len(data) // 512
    print(f'{out_path}: {blocks} blocks, {blocks * PAGE / 1024:.1f} kB of flash; IMAGE_DEF'
          f' at {at:#010x} (flags {flags:#06x}: a Secure Arm executable for the RP2350)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
