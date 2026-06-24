#!/bin/bash

# Ensure we are running Bash 3 or newer
if ((BASH_VERSINFO[0] < 3)); then
    echo "Error: This script requires Bash 3 or newer. Current version: $BASH_VERSION"
    exit 1
fi

# Enable strict mode for better error handling
set -euo pipefail

# Get the project root directory (assuming the script is in ./Tools)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
FILE_PATH="$PROJECT_ROOT/board_types.txt"

# Check if the file exists
if [ ! -f "$FILE_PATH" ]; then
    echo "Error: File 'board_types.txt' not found in project root: $PROJECT_ROOT"
    exit 1
fi

# Optional: only validate board IDs added relative to a base git ref.
# Usage: check_board_types_conflicts.sh [BASE_REF]
# When BASE_REF is provided, the full file is still scanned, but only conflicts
# that involve a newly added board are reported. This keeps the check focused on
# what a PR actually changes while still catching collisions against existing
# entries. Without BASE_REF, the entire file is checked.
BASE_REF="${1:-}"
NEW_IDS=""
if [ -n "$BASE_REF" ]; then
    # Collect the board ID (second column) of lines added in the diff.
    # Keep the result on a single space-separated line: BSD awk (macOS) rejects
    # a newline-containing value passed via -v, which breaks local PR-mode runs.
    NEW_IDS="$(git -C "$PROJECT_ROOT" diff "$BASE_REF" -- board_types.txt \
        | awk '/^\+[^+]/ { sub(/^\+/, ""); if ($1 !~ /^#/ && $2 != "") print $2 }' \
        | sort -u | tr '\n' ' ' || true)"
    if [ -z "$NEW_IDS" ]; then
        echo "No new board IDs added relative to $BASE_REF; nothing to check."
        exit 0
    fi
fi

# Process the file to detect conflicts using awk, while ignoring "# same" lines.
# Exit non-zero if any conflict is detected so CI fails loudly.
awk -v new_ids="$NEW_IDS" '
BEGIN {
    # Build a set of newly added IDs (empty set means "check everything").
    n = split(new_ids, parts, /[ \t\n]+/)
    for (i = 1; i <= n; i++) {
        if (parts[i] != "") {
            new_set[parts[i]] = 1
            only_new = 1
        }
    }
}
{
    # If the third column starts with "# same", it is a maintainer-approved
    # conflict. Still record its mapping so later lines compare against it,
    # but do not treat it as a conflict itself.
    if ($3 == "#" && $4 == "same") {
        value_map[$2] = $1
        next
    }

    # Check for conflicts on the second column
    if ($2 in value_map && value_map[$2] != $1) {
        # In new-IDs-only mode, only report conflicts involving an added board.
        if (!only_new || ($2 in new_set)) {
            print "Conflict detected: Value \"" $2 "\" is associated with both \"" value_map[$2] "\" and \"" $1 "\""
            conflicts++
        }
    }

    # Store the mapping
    value_map[$2] = $1
}
END {
    if (conflicts > 0) {
        print "ERROR: " conflicts " board ID conflict(s) detected. See above."
        exit 1
    }
    print "No board ID conflicts detected."
}
' "$FILE_PATH"
