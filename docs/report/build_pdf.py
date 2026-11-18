#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把技术报告的 Markdown 渲染为 PDF（离线：本脚本做 Markdown → HTML，Chrome 无界面打印）.

  python3 docs/report/build_pdf.py [--chrome=/usr/bin/google-chrome] [--html-only]

只支持本报告用到的 Markdown 子集：标题、段落、引用、分隔线、有序 / 无序列表（一层嵌套）、
围栏代码块、GFM 表格、独立成行的图片，以及行内的粗体、斜体、代码、链接。
输出 docs/report/report.pdf（中文正文 Noto Serif CJK SC，标题与表格 Noto Sans CJK SC）。
英文摘要 abstract_en.md 接在正文之后另起一页。
"""
import html
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))


def _blob_base():
    """仓库 origin 的 GitHub 网页地址（.../blob/main/）；取不到时返回 None."""
    try:
        url = subprocess.run(['git', '-C', REPO, 'remote', 'get-url', 'origin'],
                             capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None
    m = re.match(r'^(?:git@github\.com:|https://github\.com/)([^/]+/[^/]+?)(?:\.git)?/?$', url)
    return f'https://github.com/{m.group(1)}/blob/main/' if m else None


BLOB = _blob_base()


def link(text, target):
    """
    PDF 中的链接：仓库内的相对路径改为 GitHub 上 main 分支的地址，不把本机的绝对路径
    （Chrome 按 file:// 解析相对链接）写进 PDF；取不到远程地址时只保留文字.
    """
    if re.match(r'^(https?|mailto):', target) or target.startswith('#'):
        return f'<a href="{target}">{text}</a>'
    path, _, anchor = target.partition('#')
    rel = os.path.relpath(os.path.normpath(os.path.join(HERE, path)), REPO).replace(os.sep, '/')
    if BLOB is None or rel.startswith('..'):
        return f'<span class="link">{text}</span>'
    return f'<a href="{BLOB}{rel}{"#" + anchor if anchor else ""}">{text}</a>'


CSS = """
@page { size: A4; margin: 18mm 14mm 18mm 14mm; }
html { font-size: 10.2pt; }
body { font-family: 'Noto Serif CJK SC', 'Noto Serif CJK JP', serif; color: #111;
       line-height: 1.62; text-align: justify; }
h1, h2, h3, h4, th, .caption, table {
  font-family: 'Noto Sans CJK SC', 'Noto Sans CJK JP', sans-serif; }
h1 { font-size: 19pt; text-align: center; margin: 0 0 6pt; line-height: 1.3; }
h2 { font-size: 13.5pt; margin: 16pt 0 6pt; border-bottom: 1px solid #bbb; padding-bottom: 2pt;
     break-after: avoid; }
h3 { font-size: 11.2pt; margin: 11pt 0 4pt; break-after: avoid; }
h4 { font-size: 10.4pt; margin: 9pt 0 3pt; break-after: avoid; }
p { margin: 4pt 0; }
a, .link { color: #1a4f9c; text-decoration: none; }
code { font-family: 'Noto Sans Mono CJK SC', 'DejaVu Sans Mono', monospace; font-size: 8.8pt;
       background: #f2f1ee; padding: 0 2px; border-radius: 2px; }
pre { background: #f5f4f1; border-left: 3px solid #c9c7c0; padding: 5pt 8pt; margin: 5pt 0;
      font-size: 8.6pt; line-height: 1.45; white-space: pre-wrap; break-inside: avoid; }
pre code { background: none; padding: 0; font-size: 8.6pt; }
blockquote { margin: 6pt 0; padding: 3pt 10pt; border-left: 3px solid #86b6ef; background: #f3f7fd;
             color: #333; }
table { border-collapse: collapse; margin: 6pt auto; font-size: 8.3pt; line-height: 1.4;
        break-inside: avoid; max-width: 100%; }
th, td { border: 1px solid #c9c7c0; padding: 2.5pt 4.5pt; vertical-align: top; text-align: left; }
th { background: #efeee9; }
td:first-child, th:first-child { min-width: 4.6em; }
td code, th code { font-size: 7.8pt; }
figure { margin: 8pt 0 4pt; text-align: center; break-inside: avoid; }
figure img { max-width: 100%; max-height: 105mm; }
figure img[src*="gazebo_world"] { max-height: 70mm; }
figure img[src*="fig_architecture"] { max-height: 135mm; }
figure img.phys { max-height: none; }
.caption { font-size: 8.6pt; color: #444; text-align: center; margin: 2pt 6% 8pt; }
hr { border: none; border-top: 1px solid #ccc; margin: 10pt 0; }
ul, ol { margin: 3pt 0 3pt 0; padding-left: 20pt; }
li { margin: 1.5pt 0; }
.pagebreak { break-before: page; }
.en { font-family: 'Noto Serif', 'DejaVu Serif', serif; line-height: 1.36; font-size: 9pt; }
.en h1 { font-size: 13pt; margin-bottom: 4pt; }
.en p, .en li { margin: 2.5pt 0; }
"""


def png_dpi_size(path):
    """PNG 的像素尺寸与 pHYs 分辨率（dpi）；读不到时返回 None."""
    import struct
    try:
        with open(path, 'rb') as f:
            data = f.read(1 << 16)
    except OSError:
        return None
    if data[:8] != b'\x89PNG\r\n\x1a\n':
        return None
    w, h = struct.unpack('>II', data[16:24])
    k = data.find(b'pHYs')
    if k < 0:
        return None
    px, _, unit = struct.unpack('>IIB', data[k + 4:k + 13])
    return (w, h, px * 0.0254) if unit == 1 else None


def phys_size(src):
    """数据图（300 dpi）按实际物理尺寸排版，使图中字号与正文比例一致."""
    info = png_dpi_size(os.path.join(HERE, src))
    if not info or abs(info[2] - 300) > 1:
        return ''
    return f' class="phys" style="width: {info[0] / info[2]:.2f}in"'


def inline(s):
    codes = []

    def keep_code(m):
        codes.append('<code>' + html.escape(m.group(1)) + '</code>')
        return f'\x00{len(codes) - 1}\x00'

    s = re.sub(r'`([^`]+)`', keep_code, s)
    s = s.replace('\\*', '\x01').replace('\\_', '\x02')
    s = html.escape(s, quote=False)
    s = re.sub(r'!\[([^\]]*)\]\(([^)]+)\)', r'<img alt="\1" src="\2">', s)
    s = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', lambda m: link(m.group(1), m.group(2)), s)
    s = re.sub(r'\*\*(.+?)\*\*', r'<strong>\1</strong>', s)
    s = re.sub(r'(?<![\w*])\*(?!\s)(.+?)(?<!\s)\*(?![\w*])', r'<em>\1</em>', s)
    s = s.replace('\x01', '*').replace('\x02', '_')
    return re.sub('\x00(\\d+)\x00', lambda m: codes[int(m.group(1))], s)


def is_table_sep(line):
    return re.match(r'^\s*\|?\s*:?-{3,}:?\s*(\|\s*:?-{3,}:?\s*)*\|?\s*$', line) is not None


def split_row(line):
    line = line.strip()
    if line.startswith('|'):
        line = line[1:]
    if line.endswith('|') and not line.endswith('\\|'):
        line = line[:-1]
    return [c.strip() for c in re.split(r'(?<!\\)\|', line)]


def render(md):
    lines = md.split('\n')
    out, i, n = [], 0, len(lines)
    while i < n:
        line = lines[i]
        if not line.strip():
            i += 1
            continue
        if line.startswith('```'):
            j = i + 1
            while j < n and not lines[j].startswith('```'):
                j += 1
            out.append('<pre><code>' + html.escape('\n'.join(lines[i + 1:j])) + '</code></pre>')
            i = j + 1
            continue
        m = re.match(r'^(#{1,4})\s+(.*)$', line)
        if m:
            k = len(m.group(1))
            out.append(f'<h{k}>{inline(m.group(2))}</h{k}>')
            i += 1
            continue
        if re.match(r'^-{3,}\s*$', line):
            out.append('<hr>')
            i += 1
            continue
        m = re.match(r'^!\[([^\]]*)\]\(([^)]+)\)\s*$', line)
        if m:
            cap = ''
            j = i + 1
            while j < n and not lines[j].strip():
                j += 1
            if j < n and re.match(r'^\*[^*].*\*\s*$', lines[j]):
                cap = f'<div class="caption">{inline(lines[j].strip()[1:-1])}</div>'
                i = j
            out.append(f'<figure><img alt="{html.escape(m.group(1))}" src="{m.group(2)}"'
                       f'{phys_size(m.group(2))}>{cap}</figure>')
            i += 1
            continue
        if line.startswith('>'):
            buf = []
            while i < n and lines[i].startswith('>'):
                buf.append(lines[i][1:].strip())
                i += 1
            out.append('<blockquote>' + render('\n'.join(buf)) + '</blockquote>')
            continue
        if '|' in line and i + 1 < n and is_table_sep(lines[i + 1]):
            head = split_row(line)
            rows = []
            i += 2
            while i < n and '|' in lines[i] and lines[i].strip():
                rows.append(split_row(lines[i]))
                i += 1
            t = ['<table><thead><tr>'] + [f'<th>{inline(c)}</th>' for c in head]
            t.append('</tr></thead><tbody>')
            for r in rows:
                t.append('<tr>' + ''.join(f'<td>{inline(c)}</td>' for c in r) + '</tr>')
            t.append('</tbody></table>')
            out.append(''.join(t))
            continue
        if re.match(r'^(\s*)([-*]|\d+\.)\s+', line):
            out.append(render_list(lines, i))
            i = render_list.end
            continue
        buf = [line.strip()]
        i += 1
        while i < n and lines[i].strip() and not re.match(
                r'^(#{1,4}\s|```|>|!\[|\s*([-*]|\d+\.)\s+|-{3,}\s*$)', lines[i]) and not (
                '|' in lines[i] and i + 1 < n and is_table_sep(lines[i + 1])):
            buf.append(lines[i].strip())
            i += 1
        out.append('<p>' + inline(' '.join(buf)) + '</p>')
    return '\n'.join(out)


def render_list(lines, i):
    """从第 i 行开始的列表（支持按缩进的一层嵌套与续行）."""
    base = len(lines[i]) - len(lines[i].lstrip())
    ordered = re.match(r'^\s*\d+\.\s', lines[i]) is not None
    items, n = [], len(lines)
    while i < n:
        line = lines[i]
        if not line.strip():
            j = i + 1
            if j < n and re.match(r'^\s*([-*]|\d+\.)\s+', lines[j]) and \
                    len(lines[j]) - len(lines[j].lstrip()) >= base:
                i = j
                continue
            break
        ind = len(line) - len(line.lstrip())
        m = re.match(r'^\s*([-*]|\d+\.)\s+(.*)$', line)
        if m and ind == base:
            items.append([m.group(2), []])
            i += 1
        elif m and ind > base and items:
            sub = render_list(lines, i)
            items[-1][1].append(sub)
            i = render_list.end
        elif ind > base and items:
            items[-1][0] += ' ' + line.strip()
            i += 1
        else:
            break
    render_list.end = i
    tag = 'ol' if ordered else 'ul'
    body = ''.join(f'<li>{inline(t)}{"".join(sub)}</li>' for t, sub in items)
    return f'<{tag}>{body}</{tag}>'


def main():
    kw = dict((a[2:].split('=', 1) + [''])[:2] for a in sys.argv[1:] if a.startswith('--'))
    chrome = kw.get('chrome') or next((c for c in ('google-chrome', 'google-chrome-stable',
                                                   'chromium', 'chromium-browser')
                                       if shutil.which(c)), None)
    zh = open(os.path.join(HERE, 'README.md'), encoding='utf-8').read()
    en = open(os.path.join(HERE, 'abstract_en.md'), encoding='utf-8').read()
    # PDF 中去掉指向 PDF 自身与英文摘要文件的导航行
    zh = '\n'.join(ln for ln in zh.split('\n') if not ln.startswith('**技术报告**'))
    en = '\n'.join(ln for ln in en.split('\n') if not ln.startswith('**Abstract of the'))
    body = render(zh) + '<div class="pagebreak en">' + render(en) + '</div>'
    doc = (f'<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">'
           f'<base href="file://{HERE}/"><title>技术报告</title><style>{CSS}</style></head>'
           f'<body>{body}</body></html>')
    tmp = tempfile.mkdtemp(prefix='scr_report_')
    page = os.path.join(tmp, 'report.html')
    with open(page, 'w', encoding='utf-8') as f:
        f.write(doc)
    if 'html-only' in kw:
        print(page)
        return
    if not chrome:
        sys.exit('未找到 Chrome / Chromium，只生成了 HTML：' + page)
    out = os.path.join(HERE, 'report.pdf')
    subprocess.run([chrome, '--headless=new', '--no-sandbox', '--disable-gpu',
                    '--allow-file-access-from-files', '--no-pdf-header-footer',
                    f'--user-data-dir={os.path.join(tmp, "profile")}',
                    f'--print-to-pdf={out}', 'file://' + page],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=180)
    shutil.rmtree(tmp, ignore_errors=True)
    print('已保存', out)


if __name__ == '__main__':
    main()
