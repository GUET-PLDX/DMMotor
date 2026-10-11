#!/usr/bin/env python3
"""A short CAN frame must not refresh motor feedback freshness."""

import re
from pathlib import Path


source = (Path(__file__).resolve().parents[1] / "DMMotor.hpp").read_text()
update = source.split("LibXR::ErrorCode Update() override {", 1)[1].split("bool IsOnline() const override", 1)[0]
validate = source.split("inline bool IsFeedbackFrame(", 1)[1].split("struct MitCommand", 1)[0]

assert re.search(r"pack\.dlc\s*==\s*8", validate), "short CAN frames are not checked"
assert re.search(r"if\s*\(!DMMotorCodec::IsFeedbackFrame\(pack\)\)\s*\{\s*continue;\s*\}\s*Decode\(pack\)", update), "only valid feedback may refresh last_online_time_"
print("PASS: short CAN frame freshness guard")
