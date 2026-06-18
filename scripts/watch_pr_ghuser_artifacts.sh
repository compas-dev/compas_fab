#!/usr/bin/env bash
#
# Watch a GitHub PR for the build-cpython-components workflow run,
# download the `compas_fab_components` artifact when it succeeds, and
# install the .ghuser files into the local Rhino 8 Grasshopper
# UserObjects folder (overwriting any existing copies).
#
# Designed to be left running in the background while iterating on a PR.
# Every time a new commit lands and CI rebuilds the components, the
# script auto-installs the fresh user objects.
#
# Usage:
#   scripts/watch_pr_ghuser_artifacts.sh <PR_NUMBER> [POLL_SECONDS]
#
# Background:
#   nohup scripts/watch_pr_ghuser_artifacts.sh 123 > /tmp/pr-123.log 2>&1 &
#
# Override destination via env (e.g. for Rhino 9):
#   COMPAS_FAB_GH_USEROBJECTS="/path/to/UserObjects" scripts/watch_pr_ghuser_artifacts.sh 123

set -euo pipefail

PR="${1:?usage: $0 <PR_NUMBER> [poll_seconds]}"
POLL="${2:-30}"

WORKFLOW="build.yml"
ARTIFACT="compas_fab_components"
JOB_NAME="build-cpython-components"
DEST="${COMPAS_FAB_GH_USEROBJECTS:-$HOME/Library/Application Support/McNeel/Rhinoceros/8.0/Plug-ins/Grasshopper (b45a29b1-4343-4035-989e-044e8580d9cf)/UserObjects}"

STATE_DIR="${TMPDIR:-/tmp}/compas_fab_pr_watcher"
mkdir -p "$STATE_DIR"
STATE_FILE="$STATE_DIR/pr-$PR.last_run"

log() { printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*"; }

for cmd in gh jq; do
    command -v "$cmd" >/dev/null || { echo "missing required command: $cmd" >&2; exit 1; }
done

BRANCH=$(gh pr view "$PR" --json headRefName -q '.headRefName' 2>/dev/null || true)
[ -n "$BRANCH" ] || { log "could not resolve PR #$PR (is gh authenticated?)"; exit 1; }

log "watching PR #$PR (branch '$BRANCH'); polling every ${POLL}s"
log "destination: $DEST"

last_installed=$(cat "$STATE_FILE" 2>/dev/null || true)

install_run() {
    local run_id="$1" sha_short="$2"
    local tmp
    tmp=$(mktemp -d)
    log "downloading $ARTIFACT from run $run_id (sha $sha_short)..."
    if ! gh run download "$run_id" --name "$ARTIFACT" --dir "$tmp" 2>&1; then
        log "download failed; will retry"
        rm -rf "$tmp"
        return 1
    fi
    mkdir -p "$DEST"
    local count=0
    shopt -s nullglob
    for f in "$tmp"/*.ghuser; do
        cp -f "$f" "$DEST/" && count=$((count + 1))
    done
    shopt -u nullglob
    log "installed $count .ghuser file(s) into UserObjects"
    rm -rf "$tmp"
    return 0
}

while true; do
    run_json=$(gh run list \
        --workflow "$WORKFLOW" \
        --branch "$BRANCH" \
        --limit 1 \
        --json databaseId,status,conclusion,headSha 2>/dev/null || echo '[]')
    run_id=$(echo "$run_json" | jq -r '.[0].databaseId // empty')
    status=$(echo "$run_json" | jq -r '.[0].status // empty')
    sha=$(echo "$run_json" | jq -r '.[0].headSha // empty')
    sha_short="${sha:0:7}"

    if [ -z "$run_id" ]; then
        log "no $WORKFLOW runs on '$BRANCH' yet"
        sleep "$POLL"; continue
    fi

    if [ "$run_id" = "${last_installed:-}" ]; then
        sleep "$POLL"; continue
    fi

    if [ "$status" != "completed" ]; then
        log "run $run_id (sha $sha_short) is $status; waiting for completion..."
        gh run watch "$run_id" --exit-status >/dev/null 2>&1 || true
    fi

    job_concl=$(gh run view "$run_id" --json jobs \
        -q ".jobs[] | select(.name==\"$JOB_NAME\") | .conclusion" 2>/dev/null || echo "")
    case "$job_concl" in
        success)
            if install_run "$run_id" "$sha_short"; then
                echo "$run_id" > "$STATE_FILE"
                last_installed="$run_id"
            fi
            ;;
        "")
            log "run $run_id has no '$JOB_NAME' job (yet); will recheck"
            ;;
        *)
            log "run $run_id (sha $sha_short): job '$JOB_NAME' = $job_concl; skipping"
            echo "$run_id" > "$STATE_FILE"
            last_installed="$run_id"
            ;;
    esac

    sleep "$POLL"
done
