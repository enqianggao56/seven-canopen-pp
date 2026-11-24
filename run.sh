#!/bin/sh
set -eu
cd "$(dirname "$0")"

if [ ! -x .venv/bin/python ] || [ ! -f .venv/pyvenv.cfg ]; then
    for python in python3 python3.11 python3.12 python3.13 python3.14; do
        if "$python" -c 'import sys; sys.exit(sys.version_info < (3, 11))' 2>/dev/null; then
            "$python" -m venv .venv
            break
        fi
    done
fi

if ! .venv/bin/python -c 'import pip, sys; sys.exit(sys.version_info < (3, 11) or sys.prefix == sys.base_prefix)' 2>/dev/null; then
    echo '需要 Python 3.11 或更高版本及 venv/pip 支持。请用对应解释器执行 python3.x -m venv .venv 后重试。' >&2
    exit 1
fi

.venv/bin/python -m pip --disable-pip-version-check install --quiet -r requirements.txt
exec .venv/bin/python -m canopen_pp.main
