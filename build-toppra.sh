#!/usr/bin/env bash

set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
eigen_source="${repo_root}/deps/eigen"
eigen_build="${eigen_source}/build"
toppra_source="${repo_root}/deps/toppra/cpp"
toppra_build="${toppra_source}/build"
toppra_install="${toppra_build}/install"

if [[ ! -f "${eigen_source}/CMakeLists.txt" ]]; then
    echo "Eigen submodule is not initialized: ${eigen_source}" >&2
    echo "Run: git submodule update --init --recursive" >&2
    exit 1
fi

if [[ ! -f "${toppra_source}/CMakeLists.txt" ]]; then
    echo "TOPPRA submodule is not initialized: ${toppra_source}" >&2
    echo "Run: git submodule update --init --recursive" >&2
    exit 1
fi

cmake -S "${eigen_source}" -B "${eigen_build}" \
    -DBUILD_TESTING=OFF \
    -DEIGEN_BUILD_TESTING=OFF \
    -DEIGEN_BUILD_DOC=OFF \
    -DEIGEN_BUILD_DEMOS=OFF

cmake -S "${toppra_source}" -B "${toppra_build}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${toppra_install}" \
    -DCMAKE_CXX_STANDARD=17 \
    -DEigen3_DIR="${eigen_build}" \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTING=OFF \
    -DPYTHON_BINDINGS=OFF \
    -DBUILD_WITH_PINOCCHIO=OFF \
    -DBUILD_WITH_qpOASES=OFF \
    -DBUILD_WITH_GLPK=OFF

cmake --build "${toppra_build}" --parallel
cmake --install "${toppra_build}"

echo "TOPPRA installed in ${toppra_install}"
