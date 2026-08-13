#!/usr/bin/env bash
# Jetson field deployment entrypoint. Commands are implemented incrementally
# and always default to a non-armed operation.
set -euo pipefail

mode="${1:-}"
case "$mode" in
  stage|build|preflight|dry-run|armed)
    ;;
  *)
    printf 'Usage: %s {stage|build|preflight|dry-run|armed}\n' "$0" >&2
    exit 2
    ;;
esac

printf 'ERROR: field mode is not implemented yet: %s\n' "$mode" >&2
exit 1
