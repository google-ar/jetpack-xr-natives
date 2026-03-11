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

# Dear IMGUI native UI library

package(default_visibility = ["//visibility:public"])

licenses([
    "notice",
    "unencumbered",
])

exports_files(["LICENSE.txt"])

cc_library(
    name = "dear_imgui",
    srcs = [
        "imgui.cpp",
        "imgui_demo.cpp",
        "imgui_draw.cpp",
        "imgui_widgets.cpp",
        "imgui_tables.cpp",
        "misc/cpp/imgui_stdlib.cpp",
    ],
    hdrs = [
        "imgui.h",
        "imconfig.h",
        "imstb_textedit.h",
        "imstb_rectpack.h",
        "imstb_truetype.h",
        "imgui_internal.h",
        "misc/cpp/imgui_stdlib.h",
    ],
    features = [
        "-parse_headers",
    ],
    textual_hdrs = [
        "imgui_internal.h",
    ],
    #deps = [
    #    # We use stblib's truetype and rect_pack implementations.
    #    "@stblib//:stb_rect_pack",
    #    "@stblib//:stb_truetype",
    #],
    include_prefix = "dear_imgui",
)

cc_library(
    name = "imgui_impl_opengl3_android",
    srcs = [
        "backends/imgui_impl_android.cpp",
        "backends/imgui_impl_opengl3.cpp",
    ],
    hdrs = [
        "backends/imgui_impl_android.h",
        "backends/imgui_impl_opengl3.h",
    ],
    copts = [
        "-I./backends",
        "-I./",
        "-include ./imgui.h",
        "-x c++",
        "-Wno-pragma-once-outside-header",
        "-DIMGUI_IMPL_API=",
        "-Wno-unused-variable",
    ],
    include_prefix = "dear_imgui",
    deps = [
        ":dear_imgui",
    ],
)
