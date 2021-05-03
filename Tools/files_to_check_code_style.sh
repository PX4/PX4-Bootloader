#!/usr/bin/env bash
set -eu

ENDINGS="\.c$\|\.h$\|\.cpp$\|\.hpp$"

if [ $# -gt 0 ]
then
	exec git ls-files | grep $ENDINGS | grep - F "$1"
else
	exec git ls-files | grep $ENDINGS
fi
