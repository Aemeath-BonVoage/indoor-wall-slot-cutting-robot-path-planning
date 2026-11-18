#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shared matplotlib style for every data figure in this repository.

Usage from a figure script (any directory of the repository)::

    sys.path.insert(0, os.path.join(REPO, 'docs'))
    import figstyle as fs

    fig, ax = fs.subplots(width='single', height=2.2)
    ax.plot(x, y, **fs.line('proposed'), label='v2 (AprilTag + LiDAR EKF)')
    fs.save(fig, 'docs/experiments/<topic>/fig_name.png')   # writes .pdf and .png

Conventions
-----------
- Width: single column 3.5 in, double column 7.16 in; height chosen per figure.
- Font: Liberation Serif (metric-compatible with Times New Roman), 8 pt base,
  7 pt ticks / legends, nothing below 7 pt; mathtext uses the same family.
- Colour: Okabe-Ito colour-blind-safe palette with fixed semantic roles shared by
  all figures (see ``ROLE``); line style, marker and hatch repeat the role so the
  figures still read in greyscale.
- Export: vector PDF with embedded TrueType (Type 42) fonts plus a 300 dpi PNG with
  the same basename for the Markdown documents.
"""
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

# ---------------------------------------------------------------------------
# Sizes (inches)
# ---------------------------------------------------------------------------
SINGLE = 3.5
DOUBLE = 7.16
WIDTHS = {'single': SINGLE, 'double': DOUBLE, 'onehalf': 5.0}
DPI_PNG = 300

# ---------------------------------------------------------------------------
# Palette (Okabe & Ito) and semantic roles
# ---------------------------------------------------------------------------
BLUE = '#0072B2'
VERMILLION = '#D55E00'
GREEN = '#009E73'
ORANGE = '#E69F00'
PURPLE = '#CC79A7'
SKY = '#56B4E9'
YELLOW = '#F0E442'
GREY = '#7F7F7F'
LIGHT = '#D9D9D9'
BLACK = '#000000'

# The same role always means the same thing in every figure:
#   baseline  - v1 / first version / baseline / rule-based routes / wheel odometry
#   proposed  - v2 / current version / joint planning / visual (AprilTag + LiDAR EKF)
#   alt, alt2, alt3, alt4 - further variants or alternatives in the order they are introduced
#   neutral   - context or naive reference (e.g. per-slot partition), excluded runs
#   reference - targets, ground truth, bounds, tolerances
ROLE = {
    'baseline': VERMILLION,
    'proposed': BLUE,
    'alt': GREEN,
    'alt2': ORANGE,
    'alt3': PURPLE,
    'alt4': SKY,
    'neutral': GREY,
    'reference': BLACK,
}
LINESTYLE = {
    'baseline': (0, (4, 1.5)),
    'proposed': '-',
    'alt': (0, (4, 1, 1, 1)),
    'alt2': (0, (1, 1)),
    'alt3': (0, (6, 1.5, 1, 1.5, 1, 1.5)),
    'alt4': (0, (2, 1)),
    'neutral': (0, (1, 1.5)),
    'reference': (0, (1, 1.5)),
}
MARKER = {
    'baseline': 's',
    'proposed': 'o',
    'alt': '^',
    'alt2': 'D',
    'alt3': 'v',
    'alt4': 'P',
    'neutral': 'x',
    'reference': '+',
}
HATCH = {
    'baseline': '////',
    'proposed': None,
    'alt': '....',
    'alt2': 'xxxx',
    'alt3': '\\\\\\\\',
    'alt4': '----',
    'neutral': '||||',
    'reference': None,
}
# Ordered categorical cycle for figures without a baseline / proposed contrast
CYCLE = [BLUE, VERMILLION, GREEN, ORANGE, PURPLE, SKY, BLACK, GREY]

RC = {
    # fonts
    'font.family': 'serif',
    'font.serif': ['Liberation Serif', 'Nimbus Roman', 'TeX Gyre Termes', 'Times New Roman',
                   'DejaVu Serif'],
    'font.size': 8,
    'axes.labelsize': 8,
    'axes.titlesize': 8,
    'xtick.labelsize': 7,
    'ytick.labelsize': 7,
    'legend.fontsize': 7,
    'legend.title_fontsize': 7,
    'figure.titlesize': 8,
    'mathtext.fontset': 'custom',
    'mathtext.rm': 'Liberation Serif',
    'mathtext.it': 'Liberation Serif:italic',
    'mathtext.bf': 'Liberation Serif:bold',
    'mathtext.fallback': 'stix',
    'axes.unicode_minus': True,
    # export
    'pdf.fonttype': 42,
    'ps.fonttype': 42,
    'svg.fonttype': 'none',
    'savefig.dpi': DPI_PNG,
    'figure.dpi': 100,
    'savefig.facecolor': 'white',
    'figure.facecolor': 'white',
    'axes.facecolor': 'white',
    # layout
    'figure.constrained_layout.use': True,
    'figure.constrained_layout.h_pad': 0.02,
    'figure.constrained_layout.w_pad': 0.02,
    'figure.constrained_layout.hspace': 0.04,
    'figure.constrained_layout.wspace': 0.04,
    # axes and ticks
    'axes.linewidth': 0.6,
    'axes.edgecolor': BLACK,
    'axes.labelcolor': BLACK,
    'axes.spines.top': False,
    'axes.spines.right': False,
    'axes.titlepad': 3.0,
    'axes.labelpad': 2.0,
    'axes.titlelocation': 'left',
    'axes.prop_cycle': matplotlib.cycler(color=CYCLE),
    'axes.axisbelow': True,
    'axes.grid': False,
    'grid.color': LIGHT,
    'grid.linewidth': 0.4,
    'grid.linestyle': '-',
    'xtick.major.width': 0.6,
    'ytick.major.width': 0.6,
    'xtick.minor.width': 0.4,
    'ytick.minor.width': 0.4,
    'xtick.major.size': 2.5,
    'ytick.major.size': 2.5,
    'xtick.minor.size': 1.5,
    'ytick.minor.size': 1.5,
    'xtick.major.pad': 2.0,
    'ytick.major.pad': 2.0,
    'xtick.color': BLACK,
    'ytick.color': BLACK,
    # marks
    'lines.linewidth': 1.0,
    'lines.markersize': 3.5,
    'lines.markeredgewidth': 0.6,
    'patch.linewidth': 0.5,
    'hatch.linewidth': 0.5,
    'hatch.color': BLACK,
    'errorbar.capsize': 2.0,
    'scatter.edgecolors': 'none',
    'image.cmap': 'cividis',
    # legend
    'legend.frameon': False,
    'legend.handlelength': 2.2,
    'legend.handletextpad': 0.5,
    'legend.borderaxespad': 0.3,
    'legend.borderpad': 0.2,
    'legend.labelspacing': 0.3,
    'legend.columnspacing': 1.0,
}

# Optional callables run on each figure just before export (QA tooling can register here)
PRE_SAVE_HOOKS = []


def setup():
    """Apply the shared rcParams (called on import; safe to call again)."""
    plt.rcParams.update(RC)
    return plt


setup()


# ---------------------------------------------------------------------------
# Figure creation
# ---------------------------------------------------------------------------
def width_in(width):
    return WIDTHS[width] if isinstance(width, str) else float(width)


def figure(width='single', height=2.4, **kw):
    """Empty figure of column width with constrained layout."""
    return plt.figure(figsize=(width_in(width), height), **kw)


def subplots(nrows=1, ncols=1, width='single', height=2.4, **kw):
    """``plt.subplots`` with a column width and constrained layout."""
    return plt.subplots(nrows, ncols, figsize=(width_in(width), height), **kw)


# ---------------------------------------------------------------------------
# Style helpers
# ---------------------------------------------------------------------------
def color(role):
    return ROLE.get(role, role)


def line(role, marker=False, **kw):
    """Keyword arguments for ``ax.plot`` in a semantic role (colour + line style [+ marker])."""
    out = dict(color=ROLE[role], ls=LINESTYLE[role])
    if marker:
        out.update(marker=MARKER[role], ms=3.2, mfc='white' if role == 'baseline' else ROLE[role],
                   mec=ROLE[role])
    out.update(kw)
    return out


def marker(role, **kw):
    """Keyword arguments for a marker-only series (``ax.plot(..., ls='none')``)."""
    out = dict(color=ROLE[role], ls='none', marker=MARKER[role], ms=3.2,
               mfc='white' if role == 'baseline' else ROLE[role], mec=ROLE[role])
    out.update(kw)
    return out


def bar(role, **kw):
    """Keyword arguments for ``ax.bar`` in a semantic role (fill + hatch + thin edge)."""
    hatch = HATCH[role]
    out = dict(color=ROLE[role], edgecolor=BLACK if hatch else ROLE[role], linewidth=0.5,
               hatch=hatch)
    if role in ('baseline', 'neutral'):
        # light fill keeps the hatch visible in print and greyscale
        out.update(color='white', edgecolor=ROLE[role])
    out.update(kw)
    return out


def light_grid(ax, axis='both', which='major'):
    """Thin light grid behind the data."""
    ax.grid(True, axis=axis, which=which, color=LIGHT, lw=0.4)
    ax.set_axisbelow(True)


def label(name, unit=None):
    """Axis label with the unit in parentheses: ``label('Error', 'mm')`` -> 'Error (mm)'."""
    return f'{name} ({unit})' if unit else name


def panel_label(ax, letter, text=None, **kw):
    """Panel label '(a)' (optionally followed by a short description) at the top left."""
    s = f'({letter})' if not text else f'({letter}) {text}'
    ax.set_title(s, loc='left', fontsize=8, **kw)


def panel_labels(axes, texts=None):
    """Label a sequence of axes (a), (b), ... in order."""
    texts = texts or [None] * len(axes)
    for i, (ax, t) in enumerate(zip(axes, texts)):
        panel_label(ax, 'abcdefghijklmnopqrstuvwxyz'[i], t)


def note(ax, text, x=0.98, y=0.98, ha='right', va='top', **kw):
    """Small in-axes annotation in axes coordinates (7 pt)."""
    kw.setdefault('fontsize', 7)
    return ax.text(x, y, text, transform=ax.transAxes, ha=ha, va=va, **kw)


# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
def save(fig, path, close=True):
    """Write ``<base>.pdf`` (vector, Type 42 fonts) and ``<base>.png`` (300 dpi).

    ``path`` may end in .png, .pdf or have no extension; both files share the basename.
    """
    base, ext = os.path.splitext(path)
    if ext.lower() not in ('.png', '.pdf'):
        base = path
    for hook in PRE_SAVE_HOOKS:
        hook(fig, base)
    fig.savefig(base + '.pdf', metadata={'CreationDate': None, 'ModDate': None,
                                         'Producer': None, 'Creator': None})
    fig.savefig(base + '.png', dpi=DPI_PNG, metadata={'Software': None})
    if close:
        plt.close(fig)
    return base + '.png'


def fixed_layout(fig, left, right, bottom, top, wspace=None, hspace=None):
    """Switch a figure from constrained layout to fixed margins (inches from each edge).

    Use for rows of comparable panels so that panel widths and gutters are identical.
    ``wspace`` / ``hspace`` are gutters in inches.
    """
    w, h = fig.get_size_inches()
    fig.set_layout_engine('none')
    kw = dict(left=left / w, right=1 - right / w, bottom=bottom / h, top=1 - top / h)
    gs = None
    for ax in fig.axes:
        ss = ax.get_subplotspec()
        if ss is not None:
            gs = ss.get_gridspec()
            break
    if gs is not None:
        nrows, ncols = gs.get_geometry()
        aw = (w - left - right - (ncols - 1) * (wspace or 0)) / ncols
        ah = (h - top - bottom - (nrows - 1) * (hspace or 0)) / nrows
        if wspace is not None:
            kw['wspace'] = wspace / aw
        if hspace is not None:
            kw['hspace'] = hspace / ah
    if gs is not None:
        gs.update(**kw)          # 直接改 GridSpec，避开 subplots_adjust 对布局引擎的检查
    return fig


def plain_log(ax, axis='y'):
    """Log axis with plain-number tick labels (no 10^n superscripts below the 7 pt floor)."""
    from matplotlib.ticker import FuncFormatter, LogLocator, NullFormatter

    def fmt(v, _):
        if v <= 0:
            return ''
        return f'{v:g}' if v >= 1e-3 else f'{v:.0e}'
    a = ax.yaxis if axis == 'y' else ax.xaxis
    a.set_major_locator(LogLocator(base=10, numticks=12))
    a.set_major_formatter(FuncFormatter(fmt))
    a.set_minor_formatter(NullFormatter())


def plain_ticks(ax, axis='y'):
    """Plain-number major tick labels (no mathtext exponents) on any scale, e.g. symlog."""
    from matplotlib.ticker import FuncFormatter, NullFormatter
    a = ax.yaxis if axis == 'y' else ax.xaxis
    a.set_major_formatter(FuncFormatter(lambda v, _: f'{v:g}'))
    a.set_minor_formatter(NullFormatter())
