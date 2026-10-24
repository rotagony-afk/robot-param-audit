#!/usr/bin/env python3
"""Render the project introduction HTML to a real, printable PDF (A4, 200 dpi).

Why not a browser? The build environment for this project has no network and
blocks the IPC channels Chromium needs, so headless Chrome/Edge cannot run here.
This script therefore implements a small, honest HTML subset renderer with
Pillow. The HTML remains the single source of truth for the content.

Supported tags:
  h1 h2 h3 p ul ol li table tr td th pre div.page-break div.note footer
  inline: code strong b span br

Usage:
  python tools/html_to_pdf.py docs/作品介绍.html docs/project-introduction.pdf
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from html.parser import HTMLParser
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

DPI = 200
MM = DPI / 25.4
PAGE_W = int(round(210 * MM))
PAGE_H = int(round(297 * MM))
MARGIN_X = int(round(15 * MM))
MARGIN_TOP = int(round(17 * MM))
MARGIN_BOTTOM = int(round(15 * MM))
CONTENT_W = PAGE_W - 2 * MARGIN_X

FONT_REG = r"C:\Windows\Fonts\msyh.ttc"
FONT_BOLD = r"C:\Windows\Fonts\msyhbd.ttc"
FONT_MONO = r"C:\Windows\Fonts\consola.ttf"
FONT_MONO_B = r"C:\Windows\Fonts\consolab.ttf"
# SimSun is the classic half-width-ASCII CJK face: its CJK glyphs advance
# exactly twice the ASCII advance, which keeps ASCII art and tree diagrams
# aligned even when they contain Chinese.
FONT_CJK_MONO = r"C:\Windows\Fonts\simsun.ttc"
FONT_CJK = FONT_REG
FONT_BOLD_CJK = FONT_BOLD

INK = (31, 36, 48)
DIM = (110, 122, 136)
ACCENT = (47, 111, 208)
DEEP = (20, 48, 92)
LINE_C = (215, 224, 234)
CODE_BG = (246, 249, 252)
HEAD_BG = (238, 243, 249)
NOTE_BG = (255, 248, 230)
GOOD = (29, 122, 60)
BAD = (182, 59, 59)

NO_START = set("，。、；：？！）》」』】%·…”’,.;:!?)")


def px(pt: float) -> int:
    return max(1, int(round(pt / 72.0 * DPI)))


_FONTS: dict[tuple[str, int], ImageFont.FreeTypeFont] = {}


def get_font(path: str, size: int) -> ImageFont.FreeTypeFont:
    key = (path, size)
    if key not in _FONTS:
        _FONTS[key] = ImageFont.truetype(path, size)
    return _FONTS[key]


def is_cjk(ch: str) -> bool:
    """True for characters that occupy a full-width cell in a terminal.

    Box drawing (U+2500), arrows (U+2190) and CJK punctuation are included
    because the CJK faces carry them at double width, which is what keeps the
    ASCII-art flow diagram aligned.
    """
    code = ord(ch)
    return (0x2010 <= code <= 0x25FF) or (0x3000 <= code <= 0x9FFF) or \
           (0xF900 <= code <= 0xFAFF) or (0xFF00 <= code <= 0xFFEF)


@dataclass
class Run:
    text: str
    style: str = "normal"   # normal | strong | code


@dataclass
class Block:
    kind: str
    runs: list = field(default_factory=list)
    level: int = 0
    items: list = field(default_factory=list)
    ordered: bool = False
    rows: list = field(default_factory=list)
    widths: list = field(default_factory=list)
    text: str = ""


class DocParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.blocks: list[Block] = []
        self.runs: list[Run] = []
        self.styles: list[str] = []
        self.pre: list[str] | None = None
        self.table: list | None = None
        self.row: list | None = None
        self.cell: list | None = None
        self.widths: list[float] = []
        self.lst: Block | None = None
        self.in_li = False
        self.note = False
        self.skip = 0

    def style(self) -> str:
        for item in reversed(self.styles):
            if item:
                return item
        return "normal"

    def flush(self, kind: str = "p") -> None:
        runs = self.take()
        if runs:
            self.blocks.append(Block(kind=kind, runs=runs))

    def take(self) -> list[Run]:
        runs, self.runs = self.runs, []
        return clean(runs)

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag in ("html", "body", "head", "meta", "link"):
            return
        if tag in ("style", "script", "title"):
            self.skip += 1
            return
        if self.skip:
            return
        cls = attrs.get("class", "")
        if tag == "div":
            if "page-break" in cls:
                self.flush()
                self.blocks.append(Block(kind="pagebreak"))
            elif "note" in cls:
                self.flush()
                self.note = True
            return
        if tag in ("h1", "h2", "h3"):
            self.flush()
            self.styles.append("strong")
            return
        if tag == "p":
            return
        if tag in ("strong", "b"):
            self.styles.append("strong")
            return
        if tag == "code":
            self.styles.append("code")
            return
        if tag == "pre":
            self.flush()
            self.pre = []
            return
        if tag in ("ul", "ol"):
            self.flush()
            self.lst = Block(kind="list", ordered=(tag == "ol"))
            return
        if tag == "li":
            self.runs = []
            self.in_li = True
            return
        if tag == "table":
            self.flush()
            self.table = []
            self.widths = []
            return
        if tag == "tr":
            self.row = []
            return
        if tag in ("td", "th"):
            self.cell = []
            match = re.search(r"width:\s*([\d.]+)%", attrs.get("style", ""))
            if match and tag == "th":
                self.widths.append(float(match.group(1)))
            return
        if tag == "footer":
            self.flush()
            return
        if tag == "br":
            self.runs.append(Run("\n"))
            return

    def handle_endtag(self, tag):
        if tag in ("html", "body", "head", "meta", "link"):
            return
        if tag in ("style", "script", "title"):
            if self.skip:
                self.skip -= 1
            return
        if self.skip:
            return
        if tag in ("strong", "b", "code"):
            if self.styles:
                self.styles.pop()
            return
        if tag in ("h1", "h2", "h3"):
            runs = self.take()
            if runs:
                self.blocks.append(Block(kind="heading", runs=runs, level=int(tag[1])))
            if self.styles:
                self.styles.pop()
            return
        if tag == "p":
            if self.note:
                runs = self.take()
                if runs:
                    self.blocks.append(Block(kind="note", runs=runs))
                self.note = False
            else:
                self.flush()
            return
        if tag == "div":
            if self.note:
                runs = self.take()
                if runs:
                    self.blocks.append(Block(kind="note", runs=runs))
                self.note = False
            else:
                self.flush()
            return
        if tag == "pre":
            if self.pre is not None:
                self.blocks.append(Block(kind="pre", text="".join(self.pre)))
                self.pre = None
            return
        if tag == "li":
            runs = self.take()
            if runs and self.lst is not None:
                self.lst.items.append(runs)
            self.in_li = False
            return
        if tag in ("ul", "ol"):
            if self.lst is not None and self.lst.items:
                self.blocks.append(self.lst)
            self.lst = None
            return
        if tag in ("td", "th"):
            if self.cell is not None and self.row is not None:
                self.row.append(clean(self.cell))
            self.cell = None
            return
        if tag == "tr":
            if self.row and self.table is not None:
                self.table.append(self.row)
            self.row = None
            return
        if tag == "table":
            if self.table:
                blocks_rows = []
                for row in self.table:
                    if blocks_rows or row:
                        blocks_rows.append(row)
                self.blocks.append(Block(kind="table", rows=blocks_rows,
                                         widths=self.widths))
            self.table = None
            return
        if tag == "footer":
            self.flush("footer")
            return

    def handle_data(self, data):
        if self.skip or not data:
            return
        if self.pre is not None:
            self.pre.append(data)
            return
        text = re.sub(r"\s+", " ", data)
        if not text.strip() and not self.runs:
            return
        run = Run(text, self.style())
        if self.cell is not None:
            self.cell.append(run)
        else:
            self.runs.append(run)


def clean(runs: list[Run]) -> list[Run]:
    merged: list[Run] = []
    for run in runs:
        if not run.text:
            continue
        if merged and merged[-1].style == run.style:
            merged[-1] = Run(merged[-1].text + run.text, run.style)
        else:
            merged.append(Run(run.text, run.style))
    if merged:
        merged[0] = Run(merged[0].text.lstrip(), merged[0].style)
        merged[-1] = Run(merged[-1].text.rstrip(), merged[-1].style)
    if merged:
        text = merged[0].text
        while text.startswith(" ") and len(text) > 1:
            text = text[1:]
        merged[0] = Run(text, merged[0].style)
    return [run for run in merged if run.text]


def parse_document(path: Path) -> list[Block]:
    parser = DocParser()
    parser.feed(path.read_text(encoding="utf-8"))
    parser.close()
    parser.flush()
    return parser.blocks


# --------------------------------------------------------------------------- #
# text layout
# --------------------------------------------------------------------------- #

@dataclass
class Atom:
    text: str
    font: ImageFont.FreeTypeFont
    width: float
    space: bool = False


def atomize(runs: list[Run], size: int, code_scale: float = 0.92) -> list[Atom]:
    atoms: list[Atom] = []
    for run in runs:
        if run.style == "code":
            latin_path = FONT_MONO
            fsize = max(1, int(size * code_scale))
            cjk_path = FONT_CJK_MONO
            cjk_size = fsize
        elif run.style == "strong":
            latin_path = FONT_BOLD
            fsize = size
            cjk_path = FONT_BOLD
            cjk_size = size
        else:
            latin_path = FONT_REG
            fsize = size
            cjk_path = FONT_CJK
            cjk_size = size
        face = get_font(latin_path, fsize)
        cjk_face = get_font(cjk_path, cjk_size)
        word = ""

        def push_word() -> None:
            nonlocal word
            if word:
                atoms.append(Atom(word, face, face.getlength(word)))
                word = ""

        for ch in run.text:
            if ch == "\n":
                push_word()
                atoms.append(Atom("\n", face, 0))
            elif ch == " ":
                push_word()
                atoms.append(Atom(" ", face, face.getlength(" "), True))
            elif is_cjk(ch):
                push_word()
                atoms.append(Atom(ch, cjk_face, cjk_face.getlength(ch)))
            else:
                word += ch
        push_word()
    return atoms


def wrap(atoms: list[Atom], width: float) -> list[list[Atom]]:
    lines: list[list[Atom]] = []
    current: list[Atom] = []
    used = 0.0
    for atom in atoms:
        if atom.text == "\n":
            lines.append(current)
            current, used = [], 0.0
            continue
        if atom.space and not current:
            continue
        if used + atom.width > width and current:
            if atom.text in NO_START or atom.width > width:
                current.append(atom)
                used += atom.width
                continue
            lines.append(current)
            current, used = [], 0.0
            if atom.space:
                continue
        current.append(atom)
        used += atom.width
    if current:
        lines.append(current)
    return lines or [[]]


def draw_line(draw: ImageDraw.ImageDraw, atoms: list[Atom], x: int, y: int,
              color) -> None:
    cursor = float(x)
    for atom in atoms:
        draw.text((cursor, y), atom.text, font=atom.font, fill=color)
        cursor += atom.width


# --------------------------------------------------------------------------- #
# page renderer
# --------------------------------------------------------------------------- #

BODY = 10.5
H1 = 22.0
H2 = 14.5
H3 = 11.5
CODE = 8.7
TABLE = 9.2
FOOT = 8.5


class Renderer:
    def __init__(self) -> None:
        self.pages: list[Image.Image] = []
        self.page: Image.Image | None = None
        self.draw: ImageDraw.ImageDraw | None = None
        self.y = MARGIN_TOP
        self.pending: Block | None = None
        self.new_page()

    # -- page management --------------------------------------------------- #
    def new_page(self) -> None:
        self.flush_heading(0)
        self.page = Image.new("RGB", (PAGE_W, PAGE_H), (255, 255, 255))
        self.draw = ImageDraw.Draw(self.page)
        self.pages.append(self.page)
        self.y = MARGIN_TOP

    def ensure(self, height: float) -> None:
        if self.y + height > PAGE_H - MARGIN_BOTTOM:
            self.new_page()

    def space(self, amount: float) -> None:
        self.y += amount

    # -- primitives -------------------------------------------------------- #
    def paragraph(self, runs: list[Run], size: float = BODY, ratio: float = 1.78,
                  color=INK, indent: float = 0.0, gap: float = 5.0,
                  bullet: str | None = None) -> None:
        self.flush_heading(px(size) * ratio * 2.2)
        fsize = px(size)
        line_h = fsize * ratio
        atoms = atomize(runs, fsize)
        text_w = CONTENT_W - indent
        lines = wrap(atoms, text_w)
        for index, line in enumerate(lines):
            self.ensure(line_h)
            x = MARGIN_X + indent
            if bullet is not None and index == 0:
                self.draw.text((MARGIN_X, self.y), bullet,
                               font=get_font(FONT_REG, fsize), fill=DIM)
            draw_line(self.draw, line, int(x), int(self.y), color)
            self.y += line_h
        self.y += px(gap)

    def heading(self, block: Block) -> None:
        self.flush_heading(0)
        self.pending = block

    def flush_heading(self, min_after: float) -> None:
        """Draw a deferred heading once we know its following block fits."""
        if self.pending is None:
            return
        block, self.pending = self.pending, None
        level = block.level
        if level == 2:
            room = PAGE_H - MARGIN_BOTTOM - self.y
            if room < 0.55 * (PAGE_H - MARGIN_TOP - MARGIN_BOTTOM):
                self.new_page()
        if level == 1:
            size, colour, ratio = H1, INK, 1.35
        elif level == 2:
            size, colour, ratio = H2, DEEP, 1.45
        else:
            size, colour, ratio = H3, (28, 63, 116), 1.5
        fsize = px(size)
        self.space(px(6 if level > 1 else 10))
        runs = [Run(run.text, "strong" if run.style != "code" else "code")
                for run in block.runs]
        atoms = atomize(runs, fsize)
        lines = wrap(atoms, CONTENT_W - px(9) if level == 2 else CONTENT_W)
        block_h = len(lines) * fsize * ratio
        self.ensure(block_h + min_after + px(8))
        start_y = self.y
        for line in lines:
            draw_line(self.draw, line, MARGIN_X + (px(9) if level == 2 else 0),
                      int(self.y), colour)
            self.y += fsize * ratio
        if level == 2:
            bar_w = px(5)
            self.draw.rectangle([MARGIN_X, start_y + px(1),
                                 MARGIN_X + bar_w, self.y - px(5)], fill=ACCENT)
        self.y += px(3 if level != 3 else 1)

    def list_block(self, block: Block) -> None:
        self.flush_heading(px(BODY) * 1.78 * 2.4)
        self.space(px(1))
        for index, item in enumerate(block.items):
            mark = ("%d. " % (index + 1)) if block.ordered else "·  "
            self.paragraph(item, indent=px(16), gap=1.5, bullet=mark)
        self.space(px(4))

    def note_block(self, block: Block) -> None:
        self.flush_heading(px(BODY) * 1.72 * 2)
        fsize = px(BODY)
        line_h = fsize * 1.72
        pad = px(4)
        atoms = atomize(block.runs, fsize)
        lines = wrap(atoms, CONTENT_W - 2 * pad)
        height = len(lines) * line_h + 2 * pad + px(2)
        self.space(px(3))
        self.ensure(height + px(4))
        top = self.y
        self.draw.rectangle([MARGIN_X, top, MARGIN_X + CONTENT_W, top + height],
                            fill=NOTE_BG)
        self.draw.rectangle([MARGIN_X, top, MARGIN_X + px(2.5), top + height],
                            fill=(217, 165, 32))
        y = top + pad
        for line in lines:
            draw_line(self.draw, line, MARGIN_X + pad, int(y), (92, 70, 20))
            y += line_h
        self.y = top + height + px(5)

    def pre_block(self, block: Block) -> None:
        self.flush_heading(px(CODE) * 1.5 * 4)
        fsize = px(CODE)
        line_h = fsize * 1.5
        pad = px(5)
        inner = CONTENT_W - 2 * pad
        all_lines: list[list[Atom]] = []
        for raw in block.text.replace("\t", "    ").split("\n"):
            if not raw.strip():
                all_lines.append([])
                continue
            all_lines.extend(wrap(atomize([Run(raw, "code")], fsize, code_scale=1.0),
                                  inner))
        self.space(px(3))
        chunk: list[list[Atom]] = []
        available = PAGE_H - MARGIN_BOTTOM - MARGIN_TOP - 2 * pad

        def flush_chunk() -> None:
            nonlocal chunk
            if not chunk:
                return
            height = len(chunk) * line_h + 2 * pad
            self.ensure(height + px(2))
            top = self.y
            self.draw.rectangle([MARGIN_X, top, MARGIN_X + CONTENT_W, top + height],
                                fill=CODE_BG, outline=LINE_C)
            self.draw.rectangle([MARGIN_X, top, MARGIN_X + px(2.5), top + height],
                                fill=ACCENT)
            y = top + pad
            for line in chunk:
                draw_line(self.draw, line, MARGIN_X + pad, int(y), (27, 43, 58))
                y += line_h
            self.y = top + height + px(2)
            chunk = []

        max_lines = int(available // line_h)
        for line in all_lines:
            if len(chunk) >= max_lines:
                flush_chunk()
                self.new_page()
            chunk.append(line)
        flush_chunk()
        self.space(px(4))

    def table_block(self, block: Block) -> None:
        rows = block.rows
        if not rows:
            return
        cols = max(len(row) for row in rows)
        self.flush_heading(px(TABLE) * 1.55 * 3 + px(10))
        widths = self._column_widths(block, cols)
        fsize = px(TABLE)
        line_h = fsize * 1.55
        pad = px(3.5)
        self.space(px(3))
        for row_index, row in enumerate(rows):
            cell_lines = []
            for col in range(cols):
                runs = row[col] if col < len(row) else []
                inner = widths[col] - 2 * pad
                if not runs:
                    cell_lines.append([[]])
                    continue
                atoms = atomize(runs, fsize)
                cell_lines.append(wrap(atoms, inner))
            height = max(len(lines) for lines in cell_lines) * line_h + 2 * pad
            reserve = height if row_index else height + 2 * line_h
            if self.y + reserve > PAGE_H - MARGIN_BOTTOM:
                self.new_page()
                # re-draw the header row for readability
                if row_index > 0:
                    self._draw_row(rows[0], widths, fsize, line_h, pad, header=True)
            self._draw_row_rows(row, cell_lines, widths, fsize, line_h, pad,
                                header=(row_index == 0), height=height)
        self.space(px(6))

    def _column_widths(self, block: Block, cols: int) -> list[int]:
        declared = [w for w in block.widths if w > 0]
        if len(declared) == cols:
            total = sum(declared)
            return [int(CONTENT_W * w / total) for w in declared]
        if len(declared) == 1:
            first = int(CONTENT_W * declared[0] / 100.0)
            rest = (CONTENT_W - first) // max(cols - 1, 1)
            return [first] + [rest] * (cols - 1)
        return [CONTENT_W // cols] * cols

    def _draw_row(self, row, widths, fsize, line_h, pad, header=False) -> None:
        cell_lines = []
        for col in range(len(widths)):
            runs = row[col] if col < len(row) else []
            if not runs:
                cell_lines.append([[]])
                continue
            cell_lines.append(wrap(atomize(runs, fsize), widths[col] - 2 * pad))
        height = max(len(lines) for lines in cell_lines) * line_h + 2 * pad
        self._draw_row_rows(row, cell_lines, widths, fsize, line_h, pad,
                            header=header, height=height)

    def _draw_row_rows(self, row, cell_lines, widths, fsize, line_h, pad,
                       header, height) -> None:
        x = MARGIN_X
        top = self.y
        for col, width in enumerate(widths):
            if header:
                self.draw.rectangle([x, top, x + width, top + height], fill=HEAD_BG)
            self.draw.rectangle([x, top, x + width, top + height],
                                outline=LINE_C, width=2)
            y = top + pad
            for line in cell_lines[col]:
                draw_line(self.draw, line, int(x + pad), int(y),
                          DEEP if header else INK)
                y += line_h
            x += width
        self.y = top + height

    def footer_block(self, block: Block) -> None:
        self.space(px(6))
        fsize = px(FOOT)
        atoms = atomize([Run(run.text, run.style) for run in block.runs], fsize)
        lines = wrap(atoms, CONTENT_W)
        self.ensure(len(lines) * fsize * 1.6 + px(10))
        top = self.y
        self.draw.line([MARGIN_X, top, MARGIN_X + CONTENT_W, top], fill=LINE_C, width=2)
        self.y += px(3)
        for line in lines:
            draw_line(self.draw, line, MARGIN_X, int(self.y), DIM)
            self.y += fsize * 1.6

    # -- driver ------------------------------------------------------------ #
    def render(self, blocks: list[Block]) -> None:
        for block in blocks:
            if block.kind == "pagebreak":
                self.flush_heading(0)
                self.new_page()
            elif block.kind == "heading":
                self.heading(block)
            elif block.kind == "list":
                self.list_block(block)
            elif block.kind == "table":
                self.table_block(block)
            elif block.kind == "pre":
                self.pre_block(block)
            elif block.kind == "note":
                self.note_block(block)
            elif block.kind == "footer":
                self.flush_heading(0)
                self.footer_block(block)
            else:
                self.paragraph(block.runs, gap=2.5)
        self.flush_heading(0)
        self.finish()

    def finish(self) -> None:
        total = len(self.pages)
        fsize = px(FOOT)
        for index, page in enumerate(self.pages, start=1):
            draw = ImageDraw.Draw(page)
            text = "%d / %d" % (index, total)
            width = get_font(FONT_REG, fsize).getlength(text)
            draw.text(((PAGE_W - width) / 2, PAGE_H - MARGIN_BOTTOM + px(4)),
                      text, font=get_font(FONT_REG, fsize), fill=DIM)

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.pages[0].save(path, "PDF", resolution=DPI, save_all=True,
                           append_images=self.pages[1:])

    def preview(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        for index, page in enumerate(self.pages, start=1):
            page.save(directory / ("page-%02d.png" % index))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="HTML subset -> PDF renderer")
    parser.add_argument("html")
    parser.add_argument("pdf")
    parser.add_argument("--preview", default=None, help="dump page PNGs here")
    args = parser.parse_args(argv)
    html = Path(args.html)
    if not html.exists():
        print("html not found: " + str(html), file=sys.stderr)
        return 1
    blocks = parse_document(html)
    renderer = Renderer()
    renderer.render(blocks)
    renderer.save(Path(args.pdf))
    if args.preview:
        renderer.preview(Path(args.preview))
    print("blocks: %d  pages: %d  size: %d bytes"
          % (len(blocks), len(renderer.pages), Path(args.pdf).stat().st_size))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
