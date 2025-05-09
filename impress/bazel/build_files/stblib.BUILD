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
#   Single-file C++ image decoding and encoding libraries

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # MIT license

COPTS = select({
    "@platforms//os:windows": [],
    "//conditions:default": [
        "-Wno-unused-function",
        "$(STACK_FRAME_UNLIMITED)",
    ],
})

cc_library(
    name = "stb_image",
    srcs = ["stb_image.c"],
    hdrs = ["stb_image.h"],
    copts = COPTS,
    includes = ["."],
    include_prefix = "stblib",
)

cc_library(
    name = "stb_image_write",
    srcs = ["stb_image_write.c"],
    hdrs = ["stb_image_write.h"],
    copts = COPTS,
    includes = ["."],
    include_prefix = "stblib",
)

cc_library(
    name = "stb_rect_pack",
    hdrs = ["stb_rect_pack.h"],
    copts = COPTS,
    includes = ["."],
    include_prefix = "stblib",
)

cc_library(
    name = "stb_truetype",
    hdrs = ["stb_truetype.h"],
    copts = COPTS,
    includes = ["."],
    include_prefix = "stblib",
)
