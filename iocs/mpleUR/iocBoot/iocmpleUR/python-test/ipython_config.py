c = get_config()
c.InteractiveShellApp.extensions = ["autoreload"]
c.InteractiveShellApp.exec_lines = [
    "%autoreload complete",
    "from shell import *",
    "%pycat shell.py",
]
