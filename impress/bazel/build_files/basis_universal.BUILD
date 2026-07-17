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

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])

exports_files(["LICENSE"])

# Transcoder tables.
filegroup(
    name = "basisu_transcoder_tables",
    srcs = glob(["**/*.inc"]),
)

# C++ library for ":basisu_headers" filegroup.
cc_library(
    name = "basisu_headers_lib",
    hdrs = glob(["**/*.h", "**/*.inl"]),
    includes = [".", "encoder", "transcoder"],
    include_prefix = "basis_universal",
)

# C++ library for ":basisu_transcoder_tables" filegroup.
cc_library(
    name = "basisu_transcoder_tables_lib",
    textual_hdrs = [":basisu_transcoder_tables"],
)

basis_copts = [
    "-Ithird_party/basis_universal",
    "-Wno-implicit-fallthrough",
    "-Wno-unused-variable",
    "-Wno-unused-value",
    "-Wno-reorder",
    "-Wno-array-bounds",
    "-Wno-deprecated-builtins",
]

basis_copts_etc2_astc_only = [
    "-DBASISD_SUPPORT_ASTC=1",
    "-DBASISD_SUPPORT_ATC=0",
    "-DBASISD_SUPPORT_BC7=0",
    "-DBASISD_SUPPORT_BC7_MODE5=0",
    "-DBASISD_SUPPORT_DXT1=0",
    "-DBASISD_SUPPORT_DXT5A=0",
    "-DBASISD_SUPPORT_ETC2_EAC_A8=1",
    "-DBASISD_SUPPORT_ETC2_EAC_RG11=0",
    "-DBASISD_SUPPORT_FXT1=0",
    "-DBASISD_SUPPORT_PVRTC1=0",
    "-DBASISD_SUPPORT_PVRTC2=0",
    "-DBASISD_SUPPORT_UASTC=1",
    "-DBASISD_SUPPORT_KTX2=0",
    "-DBASISD_SUPPORT_KTX2_ZSTD=0",
]

# A "basisu" transcoder with only etc2 and astc related functionality.
# basisu_transcoder_etc2_astc adds 100K to apk size, where basis_universal adds
# 400K. BC7 transcoding is most of that increase, but BC7 is currently
# unsupported on mobile platforms.
#
# NOTE: This library uses different copts than :basis_universal so they may not
# be combined.
cc_library(
    name = "basisu_transcoder_etc2_astc",
    srcs = [
        "transcoder/basisu_transcoder.cpp",
    ],
    copts = basis_copts + basis_copts_etc2_astc_only,
    features = ["-use_header_modules"],
    deps = [
        ":basisu_headers_lib",
        ":basisu_transcoder_tables_lib",
    ],
)

# "basisu" library -- everything but main() from basisu command-line tool.
cc_library(
    name = "basis_universal",
    srcs = [
        "encoder/basisu_backend.cpp",
        "encoder/basisu_basis_file.cpp",
        "encoder/basisu_bc7enc.cpp",
        "encoder/basisu_comp.cpp",
        "encoder/basisu_enc.cpp",
        "encoder/basisu_etc.cpp",
        "encoder/basisu_frontend.cpp",
        "encoder/basisu_gpu_texture.cpp",
        "encoder/basisu_kernels_sse.cpp",
        "encoder/basisu_opencl.cpp",
        "encoder/basisu_pvrtc1_4.cpp",
        "encoder/basisu_resample_filters.cpp",
        "encoder/basisu_resampler.cpp",
        "encoder/basisu_ssim.cpp",
        "encoder/basisu_uastc_enc.cpp",
        "encoder/jpgd.cpp",
        "encoder/pvpngreader.cpp",
        "encoder/basisu_uastc_hdr_4x4_enc.cpp",
        "encoder/basisu_astc_hdr_6x6_enc.cpp",
        "encoder/basisu_astc_hdr_common.cpp",
        "encoder/basisu_astc_ldr_common.cpp",
        "encoder/basisu_astc_ldr_encode.cpp",
        "encoder/3rdparty/android_astc_decomp.cpp",
        "encoder/3rdparty/tinyexr.cpp",
        "transcoder/basisu_transcoder.cpp",
    ],
    copts = basis_copts,
    features = ["-use_header_modules"],
    deps = [
        ":basisu_headers_lib",
        ":basisu_transcoder_tables_lib",
    ],
)

# "basisu" command line tool.
cc_binary(
    name = "basisu",
    srcs = [
        "basisu_tool.cpp",
    ],
    copts = basis_copts + ["-fexceptions"],
    features = ["-use_header_modules"],
    deps = [
        ":basis_universal",
        ":basisu_headers_lib",
    ],
)
