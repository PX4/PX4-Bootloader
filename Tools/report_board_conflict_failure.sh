#!/bin/bash

# Opens a tracking issue when the weekly board conflict check fails, or appends
# a dated comment to the existing open one so repeated weekly failures collect in
# a single thread instead of spawning a new issue each week.
#
# Expects to run in GitHub Actions with the `gh` CLI authenticated via GH_TOKEN.
# Required environment:
#   GH_TOKEN       - token with issues:write (e.g. secrets.GITHUB_TOKEN)
#   RUN_URL        - URL of the failing workflow run
# Optional environment:
#   OUTPUT_FILE    - file containing the check output (default: conflict_output.txt)
#   ISSUE_TITLE    - tracking issue title (has a sensible default)

set -euo pipefail

TITLE="${ISSUE_TITLE:-Weekly board_types.txt conflict check is failing}"
OUTPUT_FILE="${OUTPUT_FILE:-conflict_output.txt}"
DATE="$(date -u +'%Y-%m-%d')"
OUTPUT="$(cat "$OUTPUT_FILE" 2>/dev/null || echo '(no output captured)')"

BODY_FILE="$(mktemp)"
{
    echo "Weekly run on **${DATE}** detected board ID conflict(s)."
    echo
    echo '```'
    echo "${OUTPUT}"
    echo '```'
    echo
    echo "Run: ${RUN_URL:-unknown}"
} > "$BODY_FILE"

# Find an existing OPEN issue with this exact title.
EXISTING="$(gh issue list --state open --search "in:title \"${TITLE}\"" \
    --json number,title --jq ".[] | select(.title == \"${TITLE}\") | .number" | head -n1)"

if [ -n "$EXISTING" ]; then
    echo "Appending recurrence to existing issue #${EXISTING}"
    gh issue comment "$EXISTING" --body-file "$BODY_FILE"
else
    echo "Creating new tracking issue"
    gh issue create --title "$TITLE" --body-file "$BODY_FILE" --label bug
fi
