#!/usr/bin/env bash
# build-recorder.sh -- Rebuild MIXR recorder libraries with protobuf v2 (T-B11)
#
# Produces:
#   mixr/lib/libmixr_recorder.a              -- recorder with regenerated protobuf
#   mixr/lib/libmixr_recorder_protobuf_v2.a  -- same objects; bridge/ and streamer/ name
#
# Prerequisites (harness/setup/apt-deps.txt):
#   sudo apt-get install -y libprotobuf-dev protobuf-compiler
#
# Usage: bash mixr/build/build-recorder.sh [--clean]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

: "${MIXR_ROOT:=${REPO_ROOT}/mixr}"
: "${MIXR_3RD_PARTY_ROOT:=${REPO_ROOT}/mixr-3rdparty}"

if [[ ! -d "${MIXR_ROOT}/src/recorder" ]]; then
    echo "ERROR: MIXR_ROOT=${MIXR_ROOT} does not contain src/recorder" >&2
    exit 1
fi

PROTO_SRC="${MIXR_ROOT}/include/mixr/recorder/protobuf/DataRecord.proto"
PROTO_CC="${MIXR_ROOT}/src/recorder/protobuf/DataRecord.pb.cc"
LIB_DIR="${MIXR_ROOT}/lib"
RECORDER_SRC="${MIXR_ROOT}/src/recorder"

CXX_BASE="${CXX:-g++}"
if command -v ccache >/dev/null 2>&1; then
    CXX_REAL="ccache ${CXX_BASE}"
    echo "ccache: enabled ($(ccache --version | head -1))"
else
    CXX_REAL="${CXX_BASE}"
    echo "ccache: not available"
fi

if [[ "${1:-}" == "--clean" ]]; then
    echo "-- Cleaning recorder build artifacts"
    rm -f "${MIXR_ROOT}/include/mixr/recorder/protobuf/DataRecord.pb.h"
    rm -f "${PROTO_CC}"
    rm -f "${RECORDER_SRC}"/protobuf/*.o "${RECORDER_SRC}"/*.o
    rm -f "${LIB_DIR}/libmixr_recorder.a"
    rm -f "${LIB_DIR}/libmixr_recorder_protobuf_v2.a"
fi

if ! command -v protoc >/dev/null 2>&1; then
    echo "ERROR: protoc not found. Install:" >&2
    echo "  sudo apt-get install -y libprotobuf-dev protobuf-compiler" >&2
    exit 1
fi
echo "-- protoc: $(protoc --version)"
# shellcheck disable=SC2016
echo "-- libprotobuf-dev: $(dpkg-query -W -f='${Version}' libprotobuf-dev 2>/dev/null || echo unknown)"

echo "-- Regenerating DataRecord.pb.h / pb.cc (proto2 API, protoc v3 compatible)"
mkdir -p "${RECORDER_SRC}/protobuf"
protoc -I "${MIXR_ROOT}/include" --cpp_out="${MIXR_ROOT}/include" "${PROTO_SRC}"
GENERATED_CC="${MIXR_ROOT}/include/mixr/recorder/protobuf/DataRecord.pb.cc"
if [[ -f "${GENERATED_CC}" ]]; then
    mv "${GENERATED_CC}" "${PROTO_CC}"
    echo "-- Moved DataRecord.pb.cc -> src/recorder/protobuf/"
fi
echo "-- Regeneration complete"

CXXFLAGS=(
    -g -O2 -fPIC
    "-I${MIXR_ROOT}/include"
    "-I${MIXR_3RD_PARTY_ROOT}/include"
    -pthread -Wall -std=c++14
    -Wno-misleading-indentation
    -Wno-unused-variable
    -Wno-unused-result
    -Wno-unused-but-set-variable
)

mkdir -p "${LIB_DIR}"

echo "-- Compiling recorder sources ($(nproc) parallel jobs)"

SRC_FILES=(
    "${RECORDER_SRC}/protobuf/DataRecord.pb.cc"
    "${RECORDER_SRC}/DataRecorder.cpp"
    "${RECORDER_SRC}/DataRecordHandle.cpp"
    "${RECORDER_SRC}/factory.cpp"
    "${RECORDER_SRC}/FileReader.cpp"
    "${RECORDER_SRC}/FileWriter.cpp"
    "${RECORDER_SRC}/InputHandler.cpp"
    "${RECORDER_SRC}/NetInput.cpp"
    "${RECORDER_SRC}/NetOutput.cpp"
    "${RECORDER_SRC}/OutputHandler.cpp"
    "${RECORDER_SRC}/PrintHandler.cpp"
    "${RECORDER_SRC}/PrintPlayer.cpp"
    "${RECORDER_SRC}/PrintSelected.cpp"
    "${RECORDER_SRC}/TabPrinter.cpp"
)

OBJS=()
pids=()
for src in "${SRC_FILES[@]}"; do
    if [[ "${src}" == "${RECORDER_SRC}/protobuf/"* ]]; then
        obj="${RECORDER_SRC}/protobuf/$(basename "${src}" .cc).o"
    else
        obj="${RECORDER_SRC}/$(basename "${src}" .cpp).o"
    fi
    OBJS+=("${obj}")
    ${CXX_REAL} "${CXXFLAGS[@]}" -c -o "${obj}" "${src}" &
    pids+=($!)
done

failed=0
for pid in "${pids[@]}"; do
    wait "${pid}" || failed=1
done
[[ ${failed} -eq 0 ]] || { echo "ERROR: compilation failed" >&2; exit 1; }
echo "-- Compiled ${#OBJS[@]} translation units"

ar rs "${LIB_DIR}/libmixr_recorder.a" "${OBJS[@]}"
echo "-- Built: ${LIB_DIR}/libmixr_recorder.a"

ar rs "${LIB_DIR}/libmixr_recorder_protobuf_v2.a" "${OBJS[@]}"
echo "-- Built: ${LIB_DIR}/libmixr_recorder_protobuf_v2.a"

echo ""
echo "=== Recorder libraries ==="
ls -lh "${LIB_DIR}/libmixr_recorder"*.a

if command -v ccache >/dev/null 2>&1; then
    echo ""
    echo "=== ccache statistics ==="
    ccache -s 2>/dev/null | tail -10
fi

echo ""
echo "build-recorder.sh: SUCCESS"
