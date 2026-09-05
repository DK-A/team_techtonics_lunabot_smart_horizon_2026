#!/usr/bin/env bash
# ==============================================================================
# LUNABOT RASPBERRY PI 4B EDGE LAUNCHER
# Simple, direct terminal execution — no systemd or background services needed!
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "========================================================================="
echo " 🌕 STARTING LUNABOT RASPBERRY PI 4B EDGE COMPUTING AGENT"
echo "========================================================================="
echo " Press Ctrl+C in this terminal at any time to disconnect."
echo ""

exec python3 "$SCRIPT_DIR/edge_agent.py" "$@"
