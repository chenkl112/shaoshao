#!/usr/bin/env bash
set -e
cd "$HOME/ccc/shaoshao"
exec python3 -u web_panel/navcar_move_web.py --port 8081
