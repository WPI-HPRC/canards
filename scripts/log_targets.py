# PlatformIO extra script: adds targets that download SD card logs from the
# board over USB (scripts/dump_log.py). They don't build or flash anything.
#
#   pio run -e marsv21 -t listlogs
#   pio run -e marsv21 -t dumplog              # previous boot's log
#   LOG_INDEX=7 pio run -e marsv21 -t dumplog  # /logs/canardsLog7.csv
#
# Set UPLOAD_PORT (upload_port in platformio.ini) to pick the port by hand.

Import("env")  # noqa: F821

import os

script = os.path.join(env.subst("$PROJECT_DIR"), "scripts", "dump_log.py")
base = '"$PYTHONEXE" "%s"' % script
port = env.subst("$UPLOAD_PORT")
if port:
    base += ' --port "%s"' % port

dump_cmd = base
if os.environ.get("LOG_INDEX"):
    dump_cmd += " --index %d" % int(os.environ["LOG_INDEX"])

env.AddCustomTarget(
    name="listlogs",
    dependencies=None,
    actions=[base + " --list"],
    title="List SD logs",
    description="List the CSV logs on the board's SD card over USB",
    always_build=True,
)
env.AddCustomTarget(
    name="dumplog",
    dependencies=None,
    actions=[dump_cmd],
    title="Download SD log",
    description="Download a CSV log from the board's SD card over USB "
                "(LOG_INDEX=N for a specific one)",
    always_build=True,
)
