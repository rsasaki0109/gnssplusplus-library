#!/bin/bash
# Automated regression test for RTK positioning
# Tests 3 datasets and validates results against expected metrics
set -e

BUILD_DIR="build"
DATA_DIR="data"
PASS=0; FAIL=0

check() {
    local name="$1" expected="$2" actual="$3" tolerance="$4"
    if python3 -c "import sys; sys.exit(0 if abs($actual - $expected) <= $tolerance else 1)"; then
        echo "  ✓ $name: $actual (expected $expected ± $tolerance)"
        PASS=$((PASS + 1))
    else
        echo "  ✗ $name: $actual (expected $expected ± $tolerance)"
        FAIL=$((FAIL + 1))
    fi
}

extract_fix_rate() {
    grep "Fix rate" "$1" | awk '{print $3}' | tr -d '%'
}

extract_rms_h() {
    grep "RMS horizontal" "$1" | awk '{print $3}'
}

echo "Building..."
cmake --build $BUILD_DIR -j$(nproc) 2>&1 | tail -1

echo ""
echo "=== Test 1: Kinematic (1.2km baseline) ==="
cd $DATA_DIR && ln -sf rover_kinematic.obs rover.obs && ln -sf base_kinematic.obs base.obs && ln -sf navigation_kinematic.nav navigation.nav && cd ..
timeout 120 ./$BUILD_DIR/examples/rtk_positioning 2>/dev/null > /tmp/rtk_test_output.txt
fix_rate=$(extract_fix_rate /tmp/rtk_test_output.txt)
rms_h=$(extract_rms_h /tmp/rtk_test_output.txt)
check "Fix rate" 100 "$fix_rate" 1
check "RMS horizontal (m)" 0.012 "$rms_h" 0.005

echo ""
echo "=== Test 2: Short Static (36m baseline) ==="
cd $DATA_DIR && ln -sf short_baseline/rover.obs rover.obs && ln -sf short_baseline/base.obs base.obs && ln -sf short_baseline/navigation.nav navigation.nav && cd ..
timeout 120 ./$BUILD_DIR/examples/rtk_positioning 2>/dev/null > /tmp/rtk_test_output.txt
fix_rate=$(extract_fix_rate /tmp/rtk_test_output.txt)
check "Fix rate" 99 "$fix_rate" 2

echo ""
echo "=== Test 3: Long Static (3.3km baseline) ==="
cd $DATA_DIR && ln -sf rover_static.obs rover.obs && ln -sf base_static.obs base.obs && ln -sf navigation_static.nav navigation.nav && cd ..
RTK_MODE=static timeout 120 ./$BUILD_DIR/examples/rtk_positioning 2>/dev/null > /tmp/rtk_test_output.txt
fix_rate=$(extract_fix_rate /tmp/rtk_test_output.txt)
check "Fix rate" 50 "$fix_rate" 15

echo ""
echo "================================"
echo "Results: $PASS passed, $FAIL failed"
[ $FAIL -eq 0 ] && echo "ALL TESTS PASSED ✓" || echo "SOME TESTS FAILED ✗"
exit $FAIL
