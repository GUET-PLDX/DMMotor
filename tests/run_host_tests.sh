#!/usr/bin/env bash
set -euo pipefail
c++ -std=c++20 -Wall -Wextra -Werror tests/dmmotor_codec_test.cpp -o /tmp/dmmotor_codec_test
/tmp/dmmotor_codec_test
