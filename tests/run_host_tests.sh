#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_dir="$(cd "${script_dir}/.." && pwd)"
workspace_root="$(cd "${module_dir}/../.." && pwd)"
libxr_dir="${workspace_root}/Middlewares/Third_Party/LibXR"
include_flags=()
while IFS= read -r -d '' include_dir; do
  include_flags+=(-isystem "${include_dir}")
done < <(find "${libxr_dir}/src" -type d -print0)
c++ -std=c++20 -Wall -Wextra -Werror \
  -DLIBXR_DEFAULT_SCALAR=float -DXR_LOG_MESSAGE_MAX_LEN=128 \
  -I"${workspace_root}/Modules/Motor" \
  -isystem "${libxr_dir}/system/linux" \
  -isystem "${libxr_dir}/lib/Eigen" \
  "${include_flags[@]}" \
  "${script_dir}/dmmotor_codec_test.cpp" -o /tmp/dmmotor_codec_test
/tmp/dmmotor_codec_test
