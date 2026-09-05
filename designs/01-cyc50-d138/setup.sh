#!/usr/bin/env bash
# setup.sh -- recreate the venv in this folder (Linux / macOS) and rerun the whole pipeline.
set -e
cd "$(dirname "$0")"
[ -d .venv ] || python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r requirements.txt
.venv/bin/python scripts/cad.py
.venv/bin/python scripts/analysis.py
.venv/bin/python scripts/check_geometry.py
.venv/bin/python scripts/render_section.py
echo "done: see cad/, analysis/, renders/"
