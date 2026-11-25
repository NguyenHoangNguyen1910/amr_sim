#!/usr/bin/env bash
set -euo pipefail

export REEMAN_COMPOSE_DIR
REEMAN_COMPOSE_DIR="$(readlink -e /reeman-run/current)"
export REEMAN_BACKUP="${REEMAN_BACKUP:-$REEMAN_COMPOSE_DIR/backup}"
export REEMAN_HTTP_PORT=8020 REEMAN_GZ_PORT=9012
export REEMAN_GAZEBO_PYTHON=/opt/venv/bin/python
exec bash src/reeman/run.sh "--${REEMAN_WORLD:-room}"
