#!/usr/bin/env python3
"""Stamp the web app with a version and optionally make it a single, standalone HTML file.

  app_build.py VERSION IN OUT             stamped copy (GitHub Pages: next to sw.js, icons, ...)
  app_build.py VERSION IN OUT --single    one self-contained file (release asset): images
                                          inlined as data: URLs, no manifest; runs opened from
                                          disk in Chrome or Edge
"""
import base64
import json
import mimetypes
import pathlib
import re
import sys


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    single = "--single" in sys.argv
    if len(args) != 3:
        sys.exit(__doc__)
    version, src, out = args
    src = pathlib.Path(src)
    html = src.read_text(encoding="utf-8")

    stamp = 'const APP_VERSION = "dev";'
    if stamp not in html:
        sys.exit("APP_VERSION placeholder not found")
    html = html.replace(stamp, f"const APP_VERSION = {json.dumps(version)};")

    if single:
        base = src.parent
        # the installable-app parts need the other files: drop them
        html = re.sub(r'\s*<link rel="(manifest|apple-touch-icon)"[^>]*>', "", html)

        def inline(m):
            path = base / m.group(2)
            if not path.is_file():
                return m.group(0)
            mime = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
            if path.suffix == ".svg":
                mime = "image/svg+xml"
            data = base64.b64encode(path.read_bytes()).decode()
            return f'{m.group(1)}="data:{mime};base64,{data}"'

        html = re.sub(r'\b(src|href)="((?:assets|icons)/[^"]+)"', inline, html)

    pathlib.Path(out).write_text(html, encoding="utf-8")
    print(f"{out}: version {version}{', single file' if single else ''}, {len(html)} bytes")


if __name__ == "__main__":
    main()
