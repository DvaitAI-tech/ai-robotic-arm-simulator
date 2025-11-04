# ai_controller.py
import re

# simple mapping between command words and motion
COMMAND_MAP = {
    "move up":      ("angle2", -2),
    "move down":    ("angle2",  2),
    "move left":    ("angle1", -2),
    "move right":   ("angle1",  2),
    "pick":         ("action", "pick"),
    "drop":         ("action", "drop"),
}

def parse_command(text):
    text = text.lower().strip()
    for cmd, action in COMMAND_MAP.items():
        if re.search(cmd, text):
            return action
    return None
