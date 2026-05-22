#!/bin/bash
# ==================================================================
# scripts/coverage.sh — one-shot coverage report runner
# Usage: ./scripts/coverage.sh [--html]
# ==================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$REPO_ROOT/out/build/coverage"
REPORT_DIR="$REPO_ROOT/out/coverage"
HTML=0

for arg in "$@"; do
    if [ "$arg" = "--html" ]; then
        HTML=1
    fi
done

echo "==> Configuring coverage build..."
cmake --preset coverage

echo "==> Building..."
cmake --build --preset coverage

echo "==> Running tests under coverage..."
ctest --test-dir "$BUILD_DIR" --output-on-failure

echo "==> Generating gcovr report..."
mkdir -p "$REPORT_DIR"

GCOVR_FILTERS=(
    --root "$REPO_ROOT"
    --exclude "$REPO_ROOT/tests/.*"
    --exclude "$REPO_ROOT/include/matiec/.*"
    --exclude "$REPO_ROOT/src/sim/.*"
    --exclude-unreachable-branches
    --exclude-throw-branches
)

gcovr "${GCOVR_FILTERS[@]}" \
      --print-summary \
      --txt "$REPORT_DIR/coverage.txt" \
      "$BUILD_DIR"

if [ "$HTML" -eq 1 ]; then
    gcovr "${GCOVR_FILTERS[@]}" \
          --html-details "$REPORT_DIR/index.html" \
          "$BUILD_DIR"
    echo "HTML report: $REPORT_DIR/index.html"
fi

echo "Text report:  $REPORT_DIR/coverage.txt"
echo "Coverage run complete."
