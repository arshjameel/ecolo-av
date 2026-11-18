"""Charts for docs/release-0.4-guide.md (static PNGs for the PDF)."""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

OUT = sys.argv[1]
BLUE, ORANGE = '#2a78d6', '#eb6834'
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e4e3df'
plt.rcParams.update({'font.size': 11, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                     'xtick.color': INK2, 'ytick.color': INK2})


def clean(ax):
    for s in ('top', 'right', 'left'):
        ax.spines[s].set_visible(False)
    ax.tick_params(length=0)


# 1. The league: ours against Nav2, three columns, each its own scale (small multiples).
fig, axes = plt.subplots(1, 3, figsize=(10, 3.4), dpi=110)
panels = [('Legs reached (%)\nhigher is better', [100, 70], '{:.0f} %'),
          ('Off the path, campus (m)\nlower is better', [0.17, 0.83], '{:.2f} m'),
          ('Over the centre line,\nUniversity Drive (%), lower is better', [0, 13], '{:.0f} %')]
for ax, (title, vals, fmt) in zip(axes, panels):
    bars = ax.bar(['Ours', 'Nav2 (gone)'], vals, color=[BLUE, ORANGE], width=0.55)
    top = max(vals) * 1.25 or 1
    ax.set_ylim(0, top)
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + top * 0.03, fmt.format(v),
                ha='center', color=INK, fontsize=11)
    ax.set_title(title, fontsize=10.5, color=INK)
    ax.set_yticks([])
    clean(ax)
fig.suptitle('The league, 16 Nov, Ravensim: ours 30 of 30 legs (3 runs), '
             'Nav2 14 of 20 (2 runs, its last score)', fontsize=10, color=INK2, y=0.02,
             va='bottom')
fig.tight_layout(rect=(0, 0.08, 1, 1))
fig.savefig(f'{OUT}/release-0.4-league.png')

# 2. Rust ports: CPU of one core, Python against Rust.
nodes = ['lidar cubes (voxels)', 'ego + look_ahead', 'sensor watchdog',
         'localization_health', 'behaviour', 'things', 'lidar_scan']
py = [25, 17, 9.9, 9.4, 9.0, 7.5, 4.8]
rs = [6, 2.4, 1.4, 1.5, 1.3, 1.5, 2.0]
fig, ax = plt.subplots(figsize=(9, 4.2), dpi=110)
y = range(len(nodes))
h = 0.38
ax.barh([i - h / 2 - 0.02 for i in y], py, height=h, color=ORANGE, label='Python')
ax.barh([i + h / 2 + 0.02 for i in y], rs, height=h, color=BLUE, label='Rust')
for i in y:
    ax.text(py[i] + 0.4, i - h / 2, f'{py[i]:g} %', va='center', color=INK2, fontsize=9.5)
    ax.text(rs[i] + 0.4, i + h / 2, f'{rs[i]:g} %', va='center', color=INK, fontsize=9.5,
            fontweight='bold')
ax.set_yticks(list(y), nodes, color=INK)
ax.invert_yaxis()
ax.set_xlabel('CPU, % of one core, Ravensim (lower is better)')
ax.xaxis.grid(True, color=GRID)
ax.set_axisbelow(True)
clean(ax)
ax.legend(frameon=False, loc='lower right')
fig.tight_layout()
fig.savefig(f'{OUT}/release-0.4-rust-cpu.png')

# 3. Kerbs: where the car stopped, before and after the step finder.
runs = ['15 Nov: no step finder', '16 Nov: step finder,\nno memory',
        '16 Nov: step finder\n+ 10 s memory']
stop = [0.0, 0.12, 0.82]   # m before the kerb; the first drove into it (no number)
fig, ax = plt.subplots(figsize=(8, 3.2), dpi=110)
bars = ax.barh(runs, stop, color=[ORANGE, BLUE, BLUE], height=0.5)
ax.axvline(0, color=INK, lw=1.5)
ax.text(-0.03, 2.55, 'the kerb', ha='right', color=INK, fontsize=10)
labels = ['drove into it (not seen)', '0.12 m before', '0.82 m before']
for b, v, t in zip(bars, stop, labels):
    ax.text(v + (0.03 if v > 0 else -0.05), b.get_y() + b.get_height() / 2, t,
            ha='left' if v > 0 else 'right',
            fontweight='bold' if v == 0 else 'normal',
            va='center', color=INK, fontsize=10)
ax.set_xlim(-1.2, 1.3)
ax.set_xticks([])
ax.invert_yaxis()
ax.set_title('A 0.3 m kerb across the road, Ravensim: where the car stopped', fontsize=11,
             color=INK)
clean(ax)
fig.tight_layout()
fig.savefig(f'{OUT}/release-0.4-kerbs.png')

# 4. The detector: ms a picture, M4 Max, 17 Nov.
runs = ['Python, PyTorch (GPU): today', 'Python, Core ML', 'Python, ONNX Runtime + Core ML',
        'Rust, ONNX Runtime + Core ML']
ms = [14.7, 11.0, 12.8, 8.0]
fig, ax = plt.subplots(figsize=(8.5, 3.0), dpi=110)
bars = ax.barh(runs, ms, color=[ORANGE, ORANGE, ORANGE, BLUE], height=0.55)
for b, v in zip(bars, ms):
    ax.text(v + 0.2, b.get_y() + b.get_height() / 2, f'{v:g} ms', va='center', color=INK,
            fontsize=10, fontweight='bold' if v == 8.0 else 'normal')
ax.set_xticks([])
ax.set_yticks(range(len(runs)), runs, color=INK)
ax.invert_yaxis()
ax.set_title('RF-DETR Nano, ms a picture (lower is better), Mac M4 Max, 17 Nov', fontsize=11,
             color=INK)
clean(ax)
fig.tight_layout()
fig.savefig(f'{OUT}/release-0.4-detector.png')

# 5. Localization: ekf against ndt_ekf, University Drive, 3 runs each.
fig, axes = plt.subplots(1, 2, figsize=(8.5, 3.0), dpi=110)
for ax, (title, vals, fmt) in zip(axes, [
        ('Sway (m/s²)', [0.064, 0.046], '{:.3f}'),
        ('Off the path (m)', [0.10, 0.03], '{:.2f} m')]):
    bars = ax.bar(['ekf (GPS): today', 'ndt_ekf: next'], vals, color=[BLUE, ORANGE],
                  width=0.55)
    top = max(vals) * 1.3
    ax.set_ylim(0, top)
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + top * 0.03, fmt.format(v), ha='center',
                color=INK)
    ax.set_title(title + ', lower is better', fontsize=10.5, color=INK)
    ax.set_yticks([])
    clean(ax)
fig.suptitle('University Drive, Ravensim, 3 runs each (the league, 16 Nov)', fontsize=10,
             color=INK2, y=0.02, va='bottom')
fig.tight_layout(rect=(0, 0.08, 1, 1))
fig.savefig(f'{OUT}/release-0.4-localization.png')
