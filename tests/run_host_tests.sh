#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_dir="$(cd "${script_dir}/.." && pwd)"
workspace_root="$(cd "${module_dir}/../.." && pwd)"
c++ -std=c++20 -Wall -Wextra -Werror \
  -I"${workspace_root}/Middlewares/Third_Party/LibXR/src/core" \
  -I"${workspace_root}/Middlewares/Third_Party/LibXR/src/utils" \
  "${script_dir}/dmmotor_codec_test.cpp" -o /tmp/dmmotor_codec_test
/tmp/dmmotor_codec_test
