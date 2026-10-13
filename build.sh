#!/bin/sh
# Linux (POSIX) build (g++ or clang++ with C++17). Output: bin/Release/dtsx-decode (override with $1).
# DTS-UHD (P2) needs the Windows-only DTSXDecoder.dll; everything else is the same as build.bat.
set -eu
cd "$(dirname "$0")"
OUT=${1:-bin/Release/dtsx-decode}
CC=${CC:-cc}
CXX=${CXX:-c++}
JOBS=${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}
OBJ=bin/obj/posix
rm -rf "$OBJ" && mkdir -p "$OBJ" "$(dirname "$OUT")"

for f in bitstream core_decoder dca_context dmix_tables exss_parser idct_fixed idct_float interpolator \
         interpolator_fixed interpolator_float lbr_decoder ta xll_decoder; do
  echo "third_party/dcadec/$f.c"
done | xargs -P "$JOBS" -I{} sh -c \
  '"$0" -O2 -w -D_USE_MATH_DEFINES -DNDEBUG -Ithird_party/dcadec -c {} -o "$1/c_$(basename {} .c).o"' "$CC" "$OBJ"

find src third_party/qrcodegen -name '*.cpp' ! -name p2_decoder.cpp | xargs -P "$JOBS" -I{} sh -c \
  '"$0" -std=c++17 -O2 -D_USE_MATH_DEFINES -DNDEBUG -I. -Isrc -Ithird_party/dcadec -Ithird_party/qrcodegen \
     -c {} -o "$1/$(echo {} | tr / _).o"' "$CXX" "$OBJ"

"$CXX" -o "$OUT" "$OBJ"/*.o -lpthread
echo "Build: $OUT"
