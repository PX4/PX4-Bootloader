#!/bin/sh

if [ -d libopencm3 ];
	then
	STATUSRETVAL=$(git submodule summary | grep -A20 -i "libopencm3" | grep "<")
	if [ -z "$STATUSRETVAL" ]; then
		echo "Checked libopencm3 submodule, correct version found"
	else
		echo ""
		echo ""
		echo "   libopencm3 sub repo not at correct version. Try 'git submodule update'"
		echo "   or follow instructions on http://px4.io/dev/git/submodules"
		echo ""
		echo "   DO NOT FORGET TO RUN 'make clean' AFTER EACH libopencm3 UPDATE!"
		echo ""
		echo ""
		echo "New commits required:"
		echo "$(git submodule summary)"
		echo ""
		exit 1
	fi
else
	git submodule init;
	git submodule update;
fi

exit 0
