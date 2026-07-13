#!/usr/bin/env bash
set -euo pipefail

HEADER="${1:-DMMotor.hpp}"

fail() {
  echo "missing: $1" >&2
  exit 1
}

need() {
  rg -q -- "$1" "$HEADER" || fail "$2"
}

need_in() {
  local pattern="$1"
  local description="$2"
  local source="$3"
  rg --multiline --pcre2 -q -- "$pattern" <<<"$source" || fail "$description"
}

forbid() {
  if rg -q -- "$1" "$HEADER"; then
    echo "forbidden: $2" >&2
    exit 1
  fi
}

need 'STARTUP_GRACE_US = 200000U' '200 ms startup grace'
need 'FEEDBACK_TIMEOUT_US = 150000U' '150 ms feedback timeout'
need 'LibXR::MicrosecondTimestamp startup_time_' 'startup timestamp'
need 'LibXR::MicrosecondTimestamp last_online_time_' 'last feedback timestamp'
need 'feedback_received_' 'first-feedback state'
forbid 'warning_timeout' 'warning freshness stage'
forbid 'stale_timeout' 'stale freshness stage'

CONSTRUCTOR="$(sed -n '/^[[:space:]]*DMMotor(/,/^[[:space:]]*void Enable() override/p' "$HEADER")"
UPDATE="$(sed -n '/^[[:space:]]*LibXR::ErrorCode Update() override {/,/^[[:space:]]*const Feedback& GetFeedback()/p' "$HEADER")"
QUEUE_LOOP="$(sed -n '/^[[:space:]]*while (recv_queue_\.Pop(pack)/,/^    }/p' <<<"$UPDATE")"
GET_FEEDBACK_BRANCH="$(sed -n '/^[[:space:]]*if (get_feedback) {/,/^    }/p' <<<"$UPDATE")"

[[ -n "$CONSTRUCTOR" ]] || fail 'DMMotor constructor extraction'
[[ -n "$UPDATE" ]] || fail 'Update extraction'
[[ -n "$QUEUE_LOOP" ]] || fail 'feedback queue loop extraction'
[[ -n "$GET_FEEDBACK_BRANCH" ]] || fail 'get-feedback branch extraction'

need_in 'startup_time_\s*(?:\(\s*LibXR::Timebase::GetMicroseconds\(\)\s*\)|=\s*LibXR::Timebase::GetMicroseconds\(\)\s*;)' \
  'constructor startup time initialization' "$CONSTRUCTOR"
need_in 'const\s+auto\s+NOW\s*=\s*LibXR::Timebase::GetMicroseconds\(\)\s*;' \
  'Update timebase sample' "$UPDATE"
need_in 'bool\s+get_feedback\s*=\s*false\s*;' \
  'get_feedback false initialization' "$UPDATE"
need_in 'while\s*\(\s*recv_queue_\.Pop\(\s*pack\s*\)\s*==\s*LibXR::ErrorCode::OK\s*\)' \
  'queue drain while Pop equals OK' "$QUEUE_LOOP"
need_in 'Decode\(\s*pack\s*\)\s*;\s*get_feedback\s*=\s*true\s*;' \
  'queue Decode and feedback flag sequence' "$QUEUE_LOOP"
need_in 'feedback_received_\s*=\s*true\s*;\s*last_online_time_\s*=\s*NOW\s*;\s*return\s+LibXR::ErrorCode::OK\s*;' \
  'get-feedback state transition' "$GET_FEEDBACK_BRANCH"
need_in 'const\s+auto\s+AGE\s*=\s*feedback_received_\s*\?\s*NOW\s*-\s*last_online_time_\s*:\s*NOW\s*-\s*startup_time_\s*;' \
  'feedback-aware age selection' "$UPDATE"
need_in 'const\s+uint64_t\s+TIMEOUT\s*=\s*feedback_received_\s*\?\s*FEEDBACK_TIMEOUT_US\s*:\s*STARTUP_GRACE_US\s*;' \
  'feedback-aware timeout selection' "$UPDATE"
need_in 'return\s+AGE\.ToMicrosecond\(\)\s*<=\s*TIMEOUT\s*\?\s*LibXR::ErrorCode::OK\s*:\s*LibXR::ErrorCode::NO_RESPONSE\s*;' \
  'inclusive freshness boundary and timeout result' "$UPDATE"

echo 'PASS: DMMotor freshness static regression checks'
