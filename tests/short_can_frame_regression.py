#!/usr/bin/env python3
"""A short CAN frame must not refresh motor feedback freshness."""

import re
from pathlib import Path


source = (Path(__file__).resolve().parents[1] / "DMMotor.hpp").read_text()
receive = source.split("static void RxCallback(", 1)[1].split("void Decode(", 1)[0]
validate = source.split("inline bool IsFeedbackFrame(", 1)[1].split("struct MitCommand", 1)[0]

assert re.search(r"pack\.dlc\s*==\s*8", validate), "short CAN frames are not checked"
assert re.search(r"if\s*\(!DMMotorCodec::IsFeedbackFrame\(pack\)\)\s*\{\s*return;\s*\}\s*self->feedback_frame_\.Store\(", receive), "only valid feedback may replace the latest frame"
print("PASS: short CAN frame freshness guard")
