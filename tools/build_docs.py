#!/usr/bin/env python3
"""
Build the docs as web pages and one PDF, from the markdown in docs/.

Purpose: docs that are easy to scan. Each page gets a red title bar, a short lead, its
sections as separate cards, tables and pictures, and a reading time. Markdown stays the
source: edit docs/<page>.md, then rebuild.

    pixi run docs              # docs/html/*.html, aav-docs.pdf and aav-release-notes.pdf
    pixi run docs --no-pdf     # just the web pages

Only the pages in PAGES are built, in that order. Open docs/html/index.html in a browser.

Markdown extras on top of the usual (headings, lists, tables, code):
- `!!! note "Title"` or `!!! warning "Title"`, body indented: a callout box.
- `[[works]]`, `[[untested]]`, `[[stub]]`, `[[missing]]`, `[[later]]`, `[[needs-you]]`:
  a status badge.
- `[[pages]]` alone on a line: the reading list, with reading times.
"""

import argparse
import datetime
import html
from pathlib import Path
import re
import shutil
import sys

import markdown

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / 'docs'
OUT = DOCS / 'html'
STYLE = DOCS / 'style.css'
WPM = 150          # an unhurried reading speed, so the times are a promise
READ_FIRST = 20    # PAGES[:20] are the reading list
LEARN_END = 35     # PAGES[20:35] are the learn pages; the rest are reference

# (file in docs/, menu title, what it's for). A name in ROOT_PAGES is a page kept at the
# repository's root instead, built the same way (none since 25 Oct: every page is in docs/).
PAGES = [
    ('index.md', 'Start here', 'The picture, and where to go next'),
    ('overview.md', 'Overview', 'Everything in ecolo-av: ours, stock, inherited, missing'),
    ('release-notes.md', 'Release notes', 'Release 0.1: what this version does, at a glance'),
    ('getting-started.md', 'Getting started', 'A new laptop to the turtle in 30 minutes (T1.1)'),
    ('first-day.md', 'Your first day', 'Install, the turtle, rqt_graph, Gazebo, Nav2'),
    ('git.md', 'Git', 'Rebasing, conflicts, worktrees; the flow is in CONTRIBUTING.md'),
    ('how-it-fits.md', 'How it fits', 'What each box does, and the safety layers'),
    ('start-up.md', 'Bottom-up', 'Each layer from the device up; start-up; testing each layer'),
    ('team.md', 'Starting points', 'One per team member: read, run, first tasks'),
    ('packages.md', 'Packages', 'What each package does, layer by layer'),
    ('modules.md', 'Every module', 'One card per package, crate and board'),
    ('work.md', 'Work', "The plan, what's done, what needs doing"),
    ('plan.md', 'The plan', 'The proposal as a to-do list: the gates, every task'),
    ('features.md', 'Features', 'What the software does, why, and who built it'),
    ('audit-friday.md', 'Audit, 16 Oct', 'The hardware audit, machine by machine'),
    ('audit-questions.md', 'Audit questions', 'What only the car can tell us'),
    ('car-day.md', 'Car day', 'The day at the car, session by session; the preflight'),
    ('tests.md', 'Every test', 'Every test, in order: sim, bench, stands, lot, the drive'),
    ('measure-the-car.md', 'Measure the car', 'The sheet for whoever measures the car'),
    ('decisions.md', 'Decisions', 'Decided, Tentative, Proposed, and Q1–Q34'),
    ('infrastructure.md', 'Infrastructure', 'The repo, Pixi, the contracts, profiles, computers'),
    ('learn-car.md', 'The car, box by box', 'Every computer, board and wire on the car'),
    ('learn-journey.md', "A command's journey", 'One push and one Nav2 goal, from hand to wheels'),
    ('faq.md', 'Questions', 'What a professor or a new teammate asks, answered'),
    ('glossary.md', 'Glossary', 'Every acronym and term, in plain words'),
    ('learn-sensors.md', 'Learn: sensors', 'Every sensor: what, how often, topic, driver'),
    ('learn-gps.md', 'Learn: GPS', 'GNSS, RTK, NavSatFix, the datum, navsat_transform'),
    ('learn-localization.md', 'Learn: localization',
     'Odometry, Kalman, frames, TF, our two EKFs, NDT and ICP'),
    ('learn-speed.md', 'Learn: speed', 'How fast are we going? Wheels, IMU, GPS, lidar, the EKF'),
    ('learn-lidar.md', 'Learn: lidar', 'Points, cubes, the costmap, named objects, NDT, AMCL'),
    ('learn-control.md', 'Learn: control', 'Feedback loops, PID, Ackermann, limits, timeouts'),
    ('controllers.md', 'Controllers',
     'Every controller, side by side: which one, where it runs, and why'),
    ('maps-and-position.md', 'Maps and position',
     'From the wheels to OpenStreetMap: frames, the datum, costmaps'),
    ('learn-nav2.md', 'Learn: Nav2', 'Planner, controller, behaviour tree, collision_monitor'),
    ('learn-can.md', 'Learn: CAN', 'CAN from two wires to our DRIVE_CMD, byte by byte'),
    ('status.md', 'Status', "What works, what's untested, what's missing"),
    ('howto-driver.md', 'Build a driver', 'A new driver, from an empty folder to the turtle'),
    ('costmaps.md', 'Costmaps', 'How Nav2 sees obstacles; draw your first map'),
    ('path-followers.md', 'Path followers', 'Regulated Pure Pursuit and MPPI, side by side'),
    ('switches.md', 'Switches', 'Every part you can swap, and pixi run pick'),
    ('trials.md', 'Trials', 'New path followers and AMCL, beside the main ones'),
    ('a-to-b.md', 'A to B', 'Drive from A to B and stop: three ways, and the point driver'),
    ('routes.md', 'Street routes', 'A destination to GPS waypoints along real streets (Valhalla)'),
    ('perception.md', 'Perception', 'Cameras, YOLO, stop signs, lanes and radar'),
    ('perception-and-hud-map.md', 'Perception options',
     'Which model for which job, the lidar, a map on the HUD'),
    ('camera-pipeline.md', 'Camera pipeline',
     "The Pi's H.264 stream to detections on the Orin: settings, measured delay"),
    ('camera-calibration.md', 'Camera calibration',
     'At the car: print the board, run camera_calibration on the stream, file the result'),
    ('proposal.md', 'Proposal draft', 'The 15 Oct draft: every part, built or left, how sure'),
    ('team-todo.md', 'Team tasks', 'The work list from before the audit, by area (A-E)'),
    ('older-code.md', 'Older code', "The previous teams' repos: what we have, what's missing"),
    ('ros2.md', 'ROS 2', 'Nodes, topics and tools; stock packages vs our code'),
    ('languages.md', 'Languages', 'Python, C++ and Rust: what goes where, and why'),
    ('run.md', 'Run it', 'Mac or Linux: install, build, every command'),
    ('mac.md', 'Your Mac', 'Set up the Mac, join the car, drive and watch'),
    ('foxglove.md', 'Watch from a laptop',
     'Foxglove and Lichtblick: connect, the layouts, bandwidth'),
    ('simulation.md', 'Simulation', 'Gazebo, replay, CARLA, and when nothing moves'),
    ('sim-and-car.md', 'Sim and the car', "What's simulated, what's real, and why"),
    ('full-simulation.md', 'Full simulation',
     'The whole car in Gazebo: the pipeline, the actuators'),
    ('carla.md', 'CARLA', "Photo-real towns and Carleton's streets: UE5 vs UE4, the twin"),
    ('bench.md', 'Bench mode', "The car's software on one computer, with stand-ins"),
    ('actuator-tests.md', 'Actuator tests', 'Measure how the car moves: on stands, in a lot'),
    ('learn-state-space.md', 'Learn: state space',
     'From a drawing to a car simulator: matrices, poles, Bode, roll'),
    ('vehicle-model.md', 'The car as a linear model',
     'Transfer functions, state space, poles and margins (SYSC 3600)'),
    ('ghost-obstacles.md', 'Ghost obstacles', 'A pretend person only Nav2 sees: test stopping'),
    ('geofence.md', 'The geofence', "An area the computer's drivers may not leave, without Nav2"),
    ('sources-and-targets.md', 'Sources and targets',
     'The sensor mux and the output mux: simulated data never drives the car'),
    ('mcp.md', 'AI apps (MCP)', 'Claude and other AI apps: read the project and the graph'),
    ('plain-english.md', 'Plain English', 'Type what the car should do; a model plans it'),
    ('model-comparison.md', 'Newer models', 'YOLO26 and RF-DETR vs YOLO11; newer local LLMs'),
    ('firmware.md', 'Firmware', 'ros vs firmware, the CAN contract, the spine'),
    ('can-plan.md', 'CAN plan', "Today's wiring (CAN for sensors), and the KEFC plan"),
    ('rust-firmware.md', 'Rust firmware', 'Rust on the chips: layers, frameworks, toolchains'),
    ('esp32-bench.md', 'ESP32 first flash', 'The Rust ESP32 on the bench: flash, check, undo'),
    ('platforms.md', 'Platforms', 'Ubuntu, the Orin and Pis, networking'),
    ('bandwidth.md', 'Bandwidth', 'How much data each sensor sends, and which wires have room'),
    ('car-install.md', 'From a USB stick', 'Our Jazzy on the car offline, no sudo: the kit'),
    ('car-screens.md', 'The two screens', 'The HUD and the map, full screen at boot'),
    ('car-photos.md', 'Car photos', 'What the car has, from photos (14 Oct), and the audit list'),
    ('conventions.md', 'Conventions', 'Names, package layout and code style'),
    ('naming.md', 'Topic areas', 'Topic names by area (Decided), and why the EKFs meet in TF'),
    ('interfaces.md', 'Topics', 'Every topic, service, action and frame'),
    ('car_measurements.md', 'Car numbers', 'vehicle.yaml, and how to measure the car'),
    ('hud.md', 'HUD', 'The in-car screen and the map window: Rust and raylib'),
]
BUILT = {name for name, _, _ in PAGES}
# The pages kept at the root: their name here -> their file, from the root.
ROOT_PAGES = {}
# Links to them from docs/ (../FILE.md) go to the built page.
ROOT_LINKS = {'../' + file: name for name, file in ROOT_PAGES.items()}
RELEASE = 'release-notes.md'   # also printed on its own, with a cover, to share
RELEASE_PDF = 'aav-release-notes.pdf'
# The release notes' own PDF: a running footer, and the cover on a page of its own.
RELEASE_STYLE = """
@page {
  @bottom-left { content: "AAV 2026-27 autonomy stack · release notes"; font-size: 9px;
                 color: #777; font-family: Verdana, "DejaVu Sans", Tahoma, Arial, sans-serif; }
  @bottom-center { content: none; }
  @bottom-right { content: counter(page) " of " counter(pages); font-size: 9px; color: #777;
                  font-family: Verdana, "DejaVu Sans", Tahoma, Arial, sans-serif; }
}
@page :first { @bottom-left { content: none; } @bottom-right { content: none; } }
.release .cover { break-after: page; }
"""
BADGES = ('works', 'untested', 'stub', 'missing', 'later', 'needs-you')

LIST_ITEM = re.compile(r'\s*([-*+]|\d+\.)\s')
PDF_CONTENTS_NOTE = '<p>The contents, with page numbers, are on the first page.</p>'


def slug(name):
    return name[:-3]


def minutes(text):
    prose = re.sub(r'```.*?```', '', text, flags=re.S)
    return max(1, round(len(re.findall(r"[A-Za-z0-9']+", prose)) / WPM))


def loosen(text):
    """Python-Markdown wants a blank line before a list or table; GitHub doesn't. Add it."""
    out, fence = [], False
    for line in text.splitlines():
        if line.lstrip().startswith('```'):
            fence = not fence
        prev = out[-1] if out else ''
        starts_block = LIST_ITEM.match(line) or line.startswith('|')
        prev_is_text = (prev.strip() and not prev.startswith((' ', '|', '\t'))
                        and not LIST_ITEM.match(prev))
        if not fence and starts_block and not line.startswith(' ') and prev_is_text:
            out.append('')
        out.append(line)
    return '\n'.join(out) + '\n'


def callouts(text):
    """
    Indent each `!!! note` box's body by two spaces, not four.

    The pages indent a box's body by four spaces, as MkDocs does; with tab_length 2
    that read as a code block inside the box (found 24 Oct). Fences inside move with it.
    """
    out, inside = [], False
    for line in text.splitlines():
        if line.startswith('!!! '):
            inside = True
        elif inside and line.strip() and not line.startswith('    '):
            inside = False
        out.append(line[2:] if inside and line.startswith('    ') else line)
    return '\n'.join(out) + '\n'


def convert(text):
    md = markdown.Markdown(
        extensions=['tables', 'attr_list', 'def_list', 'md_in_html', 'sane_lists', 'toc',
                    'admonition', 'pymdownx.superfences'],
        tab_length=2)  # our lists nest by two spaces
    return md.convert(loosen(callouts(text)))


def fix_links(body, pdf=False):
    """
    Point the links at the built pages.

    Links in the markdown are relative to docs/. Pages become .html; anything else points
    back at the file in the repository, one folder up from docs/html/.
    """
    def fix(m):
        attr, url = m.group(1), m.group(2)
        if re.match(r'[a-z]+:|#|/', url):
            return m.group(0)
        path, _, frag = url.partition('#')
        path = ROOT_LINKS.get(path, path)
        if path in BUILT:
            if pdf:
                return f'{attr}="#{slug(path)}{"--" + frag if frag else ""}"'
            return f'{attr}="{slug(path)}.html{"#" + frag if frag else ""}"'
        return f'{attr}="../{url}"'
    return re.sub(r'(href|src)="([^"]*)"', fix, body)


def from_root(text):
    """
    Make a root page's links relative to docs/, as the other pages' are.

    docs/x.md becomes x.md; tools/x.py becomes ../tools/x.py.
    """
    def fix(m):
        url = m.group(1)
        if re.match(r'[a-z]+:|#|/', url):
            return m.group(0)
        return '](' + (url[5:] if url.startswith('docs/') else '../' + url) + ')'
    return re.sub(r'\]\(([^)\s]+)\)', fix, text)


def source_of(name):
    """Return the page's markdown file, from the repository's root."""
    return ROOT_PAGES.get(name, f'docs/{name}')


def badges(body):
    """[[works]] -> a coloured badge, everywhere except inside code."""
    parts = re.split(r'(<pre.*?</pre>|<code>.*?</code>)', body, flags=re.S)
    pattern = re.compile(r'\[\[(' + '|'.join(BADGES) + r')\]\]')
    for i in range(0, len(parts), 2):
        parts[i] = pattern.sub(
            lambda m: f'<span class="badge b-{m.group(1)}">{m.group(1).replace("-", " ")}</span>',
            parts[i])
    return ''.join(parts)


def reading_list(pages, pdf=False):
    rows = []
    for i, (name, title, purpose, mins) in enumerate(pages):
        if i in (READ_FIRST, LEARN_END):
            label = ('Learn: the car and the code, step by step' if i == READ_FIRST
                     else 'Reference: look things up here')
            rows.append(f'<tr><td colspan="{5 if pdf else 4}" class="group">'
                        f'{label}</td></tr>')
        href = f'#{slug(name)}' if pdf else f'{slug(name)}.html'
        num = str(i + 1) if i < READ_FIRST else ''
        page = f'<td class="time"><a class="pg" href="{href}"></a></td>' if pdf else ''
        rows.append(f'<tr><td class="num">{num}</td>'
                    f'<td><a href="{href}"><b>{html.escape(title)}</b></a></td>'
                    f'<td>{html.escape(purpose)}</td>'
                    f'<td class="time">{mins} min</td>{page}</tr>')
    head = '<th>Page</th>' if pdf else ''
    return ('<div class="table-wrap"><table class="pages"><tr><th></th><th>Section</th>'
            f'<th>Read it for</th><th>Time</th>{head}</tr>' + ''.join(rows) + '</table></div>')


def split_page(body):
    """-> (title, intro, [(h2 id, h2 text, section html)])."""
    m = re.search(r'<h1[^>]*>(.*?)</h1>', body, flags=re.S)
    title = re.sub(r'<[^>]+>', '', m.group(1)) if m else ''
    if m:
        body = body[:m.start()] + body[m.end():]
    parts = re.split(r'(?=<h2[ >])', body)
    sections = []
    for part in parts[1:]:
        h = re.match(r'<h2(?: id="([^"]*)")?>(.*?)</h2>', part, flags=re.S)
        sections.append((h.group(1) or '', re.sub(r'<[^>]+>', '', h.group(2)), part))
    return title, parts[0].strip(), sections


def page_body(intro, sections):
    out = [f'<div class="lead">{intro}</div>'] if intro else []
    out += [f'<section class="card">{s}</section>' for _, _, s in sections]
    return '\n'.join(out)


def menu(here):
    links = []
    for i, (name, title, _) in enumerate(PAGES):
        if i == READ_FIRST:
            links.append('<span class="sep">Learn:</span>')
        if i == LEARN_END:
            links.append('<span class="sep">Reference:</span>')
        cls = ' class="here"' if name == here else ''
        links.append(f'<a{cls} href="{slug(name)}.html">{html.escape(title)}</a>')
    return '\n'.join(links)


def prev_next(i):
    out = []
    if i > 0:
        name, title, _ = PAGES[i - 1]
        out.append(f'<a class="prev" href="{slug(name)}.html">'
                   f'<small>Previous</small>← {html.escape(title)}</a>')
    else:
        out.append('<span></span>')
    if i + 1 < len(PAGES):
        name, title, _ = PAGES[i + 1]
        out.append(f'<a class="next" href="{slug(name)}.html">'
                   f'<small>Next</small>{html.escape(title)} →</a>')
    return '\n'.join(out)


def web_page(i, source, title, mins, intro, sections):
    toc = ''.join(f'<a href="#{sid}">{html.escape(text)}</a>' for sid, text, _ in sections)
    md = source_of(source)
    return f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html.escape(title)} · AAV 2026–27</title>
<link rel="stylesheet" href="style.css">
</head><body>
<!-- Generated from {md} by tools/build_docs.py. Edit the markdown, not this file. -->
<header class="bar"><div class="kicker">AAV 2026–27 · AUTONOMY STACK</div>
<h1>{html.escape(title)}</h1>
<div class="meta">{mins} min read · {len(sections)} sections</div></header>
<nav class="menu">{menu(source)}</nav>
<main>
<nav class="toc"><span>On this page:</span>{toc}</nav>
{page_body(intro, sections)}
<nav class="prevnext">{prev_next(i)}</nav>
</main>
<footer>Source: <code>{md}</code>. Edit it, then run <code>pixi run docs</code>.</footer>
</body></html>
"""


def pdf_article(name, title, intro, sections):
    """One page of the PDF: ids get the page's name in front, so they stay unique."""
    body = page_body(intro, sections)
    body = re.sub(r'id="([^"]+)"', lambda m: f'id="{slug(name)}--{m.group(1)}"', body)
    body = re.sub(r'href="#([^"]+)"', lambda m: f'href="#{slug(name)}--{m.group(1)}"', body)
    return (f'<article class="page" id="{slug(name)}"><header class="bar">'
            f'<div class="kicker">AAV 2026–27 · AUTONOMY STACK</div><h1>{html.escape(title)}</h1>'
            f'</header>{body}</article>')


def write_pdf(articles, listing, out):
    from weasyprint import HTML   # only the docs environment has it
    today = datetime.date.today().strftime('%-d %B %Y')
    cover = f"""<section class="cover"><div class="kicker">CARLETON UNIVERSITY · AAV CAPSTONE</div>
<h1>The AAV 2026–27 autonomy stack</h1>
<p class="sub">How it fits together, how to run it, and what's left</p>
{listing}
<p class="date">Built {today} from the markdown in the repository's docs/ folder</p></section>"""
    doc = (f'<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">'
           f'<link rel="stylesheet" href="style.css"></head><body class="print">'
           f'{cover}{"".join(articles)}</body></html>')
    HTML(string=doc, base_url=str(OUT) + '/').write_pdf(out)


def write_release_pdf(title, intro, sections, out):
    """Print the release notes on their own: a cover with the pitch, then the sections."""
    from weasyprint import HTML   # only the docs environment has it
    inside = ''.join(f'<li><a href="#{sid}">{html.escape(text)}</a>'
                     f'<a class="pg" href="#{sid}"></a></li>' for sid, text, _ in sections)
    cover = ('<section class="cover"><div class="kicker">CARLETON UNIVERSITY · '
             f'AUTONOMOUS VEHICLE CAPSTONE</div><h1>{html.escape(title)}</h1>{intro}'
             f'<h3 class="inside-title">Inside</h3><ol class="inside">{inside}</ol></section>')
    doc = ('<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">'
           f'<link rel="stylesheet" href="style.css"><style>{RELEASE_STYLE}</style></head>'
           f'<body class="print release">{cover}{page_body("", sections)}</body></html>')
    HTML(string=doc, base_url=str(OUT) + '/').write_pdf(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--no-pdf', action='store_true', help='build the web pages only')
    args = ap.parse_args()

    OUT.mkdir(exist_ok=True)
    shutil.copyfile(STYLE, OUT / 'style.css')
    pages = []
    for name, title, purpose in PAGES:
        text = (ROOT / source_of(name)).read_text(encoding='utf-8')
        if name in ROOT_PAGES:
            text = from_root(text)
        pages.append((name, title, purpose, minutes(text), convert(text)))
    listing = [(n, t, p, m) for n, t, p, m, _ in pages]

    articles, release = [], None
    for i, (name, _, _, mins, body) in enumerate(pages):
        body = badges(body).replace('<table>', '<div class="table-wrap"><table>') \
                           .replace('</table>', '</table></div>')
        web = fix_links(body).replace('<p>[[pages]]</p>', reading_list(listing))
        title, intro, sections = split_page(web)
        (OUT / f'{slug(name)}.html').write_text(
            web_page(i, name, title, mins, intro, sections), encoding='utf-8')
        if not args.no_pdf:
            printed = body.replace('<p>[[pages]]</p>', PDF_CONTENTS_NOTE)
            articles.append(pdf_article(name, *split_page(fix_links(printed, pdf=True))))
            if name == RELEASE:
                release = split_page(fix_links(body, pdf=True))
    print(f'{len(pages)} pages -> {OUT.relative_to(ROOT)}/')

    if not args.no_pdf:
        pdf = OUT / 'aav-docs.pdf'
        write_pdf(articles, reading_list(listing, pdf=True), pdf)
        print(f'PDF -> {pdf.relative_to(ROOT)} ({pdf.stat().st_size // 1024} KB)')
        if release:
            pdf = OUT / RELEASE_PDF
            write_release_pdf(*release, pdf)
            print(f'PDF -> {pdf.relative_to(ROOT)} ({pdf.stat().st_size // 1024} KB)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
