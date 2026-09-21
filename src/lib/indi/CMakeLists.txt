# SPDX-License-Identifier: BSD-3-Clause
px4_add_library(IndiControl IndiControl.cpp)
target_compile_options(IndiControl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
