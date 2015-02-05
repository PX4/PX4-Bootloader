#!/usr/bin/env bash
set -eu
failed=0
for fn in $(find . -path './libopencm3' -prune -o -name '*.c' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -type f); do
  if [ -f "$fn" ];
  then
    ./Tools/fix_code_style.sh --quiet < $fn > $fn.pretty
    diff -u $fn $fn.pretty || failed=1
    rm $fn.pretty
  fi
done

if [ $failed -eq 0 ]; then
    echo "Format checks passed"
    exit 0
else
    echo "Format checks failed; please run 'make format'"
    exit 1
fi
