# Vendored xsimd

- Upstream: https://github.com/xtensor-stack/xsimd
- Tag: 13.2.0
- License: BSD-3-Clause (see LICENSE)
- What is vendored: the `include/` tree only (header-only library)
- Why: CMake must not FetchContent or download at configure time

Update by replacing this directory with another tagged release's `include/` and LICENSE.
