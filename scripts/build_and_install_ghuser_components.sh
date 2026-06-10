#!/usr/bin/env bash
#
# Build the CPython Grasshopper user objects locally and install the
# generated .ghuser files into the local Rhino 8 Grasshopper UserObjects
# folder (overwriting any existing copies).
#
# Usage:
#   scripts/build_and_install_ghuser_components.sh
#
# Override destination via env (e.g. for Rhino 9):
#   COMPAS_FAB_GH_USEROBJECTS="/path/to/UserObjects" scripts/build_and_install_ghuser_components.sh
#
# Each component's docstring header carries a `COMPAS FAB v<x>` marker line.
# This script stamps the build time onto that line in the embedded script before
# building, then restores the sources, so every built component records when it
# was generated without dirtying the working tree. The .ghuser stores the script
# in a binary archive, so the stamp must go in before the build, not after.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPONENTS="$ROOT/src/compas_fab/ghpython/components_cpython"
SOURCE="$COMPONENTS/ghuser"
DEST="${COMPAS_FAB_GH_USEROBJECTS:-$HOME/Library/Application Support/McNeel/Rhinoceros/8.0/Plug-ins/Grasshopper (b45a29b1-4343-4035-989e-044e8580d9cf)/UserObjects}"

# Build-time stamp appended to the version marker (e.g. "(built 2026-06-08 14:23 UTC)").
STAMP="${COMPAS_FAB_GH_STAMP:-built $(date -u '+%Y-%m-%d %H:%M UTC')}"

log() { printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*"; }

command -v invoke >/dev/null || { echo "missing required command: invoke" >&2; exit 1; }

cd "$ROOT"

# Collect every component's code.py so we can stamp and reliably restore them.
CODE_FILES=()
while IFS= read -r f; do CODE_FILES+=("$f"); done \
    < <(find "$COMPONENTS" -mindepth 2 -maxdepth 2 -name code.py)

# Restore originals from their backups no matter how the script exits (success,
# failure, or interrupt), so a stamped header is never left behind in the source.
restore_sources() {
    for f in "${CODE_FILES[@]}"; do
        [ -f "$f.stampbak" ] && mv -f "$f.stampbak" "$f"
    done
}
trap restore_sources EXIT

log "stamping ${#CODE_FILES[@]} component(s) with: $STAMP"
for f in "${CODE_FILES[@]}"; do
    cp -p "$f" "$f.stampbak"
    # Append the stamp to the version marker line, tolerating any existing suffix.
    # The stamp is passed via the environment and the replacement is evaluated
    # (/e), so its contents can never break the substitution.
    STAMP="$STAMP" perl -pi -e 's/^(COMPAS FAB v[0-9][^(\n]*?)[ \t]*(\(built [^)]*\))?[ \t]*$/"$1 (" . $ENV{STAMP} . ")"/e' "$f"
done

log "building CPython Grasshopper user objects..."
PYTHONNET_RUNTIME=mono invoke build-cpython-ghuser-components

mkdir -p "$DEST"

count=0
shopt -s nullglob
for f in "$SOURCE"/*.ghuser; do
    cp -f "$f" "$DEST/" && count=$((count + 1))
done
shopt -u nullglob

if [ "$count" -eq 0 ]; then
    echo "no .ghuser files found in $SOURCE" >&2
    exit 1
fi

log "installed $count .ghuser file(s) into UserObjects"
log "destination: $DEST"
