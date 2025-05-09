# Copyright 2025 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Description:
#
# Library for performing reflection on SPIR-V and disassembling SPIR-V bytecode.

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

exports_files(["LICENSE"] + glob(["LICENSES/*"]))

COMMON_COPTS = [
    "-fexceptions",
    "-Wno-implicit-fallthrough",
]

cc_library(
    name = "spirv_cross_lib",
    srcs = [
        "GLSL.std.450.h",
        "spirv_cfg.cpp",
        "spirv_cpp.cpp",
        "spirv_cross.cpp",
        "spirv_cross_c.cpp",
        "spirv_cross_parsed_ir.cpp",
        "spirv_cross_util.cpp",
        "spirv_glsl.cpp",
        "spirv_hlsl.cpp",
        "spirv_msl.cpp",
        "spirv_parser.cpp",
        "spirv_reflect.cpp",
    ],
    hdrs = [
        "spirv.h",
        "spirv.hpp",
        "spirv_cfg.hpp",
        "spirv_common.hpp",
        "spirv_cpp.hpp",
        "spirv_cross.hpp",
        "spirv_cross_c.h",
        "spirv_cross_containers.hpp",
        "spirv_cross_error_handling.hpp",
        "spirv_cross_parsed_ir.hpp",
        "spirv_cross_util.hpp",
        "spirv_glsl.hpp",
        "spirv_hlsl.hpp",
        "spirv_msl.hpp",
        "spirv_parser.hpp",
        "spirv_reflect.hpp",
    ],
    copts = COMMON_COPTS,
    includes = ["."],
)
