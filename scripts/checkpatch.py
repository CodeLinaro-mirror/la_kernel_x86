#!/usr/bin/env python
#
# This is a simple wrapper used by .git/hooks/post-commit to run
# kernel's checkpatch.pl.

import os
import subprocess
import sys

cmdargs=sys.argv[1:]

subprocess.call(
	[os.path.join("scripts", "checkpatch.pl")] + cmdargs
)
