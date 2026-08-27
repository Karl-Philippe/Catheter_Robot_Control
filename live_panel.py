"""
Shortcut: run dual_control.py with the live screen on.

The panel is not a separate use case -- it is a switch on the replay scripts.
Set LIVE_PANEL = True in control.py or dual_control.py and run those directly,
or use this file to get the dual-channel one without editing anything:

    python3 live_panel.py

Everything it replays (log path, columns, interleave, retract, initial
orientation) comes from dual_control.py's SETTINGS block.
"""

import dual_control


def main():
    dual_control.LIVE_PANEL = True
    return dual_control.main()


if __name__ == "__main__":
    main()
