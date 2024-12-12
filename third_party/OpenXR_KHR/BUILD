# Copyright 2024 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load(
    ":flags_and_opts.bzl",
    "OPENXR_ANDROID_COPTS",
    "OPENXR_ANDROID_LINKOPTS",
)

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])

exports_files(["LICENSE"])

cc_library(
    name = "openxr_headers_androidxr",
    hdrs = glob(["generated/include/openxr/*.h"]),
    includes = [
        "generated/include/openxr",
    ],
    strip_include_prefix = "//third_party/OpenXR_KHR/generated/include",
)

cc_library(
    name = "openxr_loader",
    srcs = glob([
        "generated/src/**",
        "generated/include/openxr/**",
        "src/common/**",
        "src/loader/*.h",
        "src/loader/*.hpp",
        "src/loader/*.cpp",
    ]),
    copts = [
        "-DAPI_NAME=\"OpenXR\"",
        "-DDISABLE_STD_FILESYSTEM",
        "-DXRAPI_DLL_EXPORT",
        "-Dopenxr_loader_EXPORTS",
        "-DNDEBUG",
        "-fexceptions",
        "-fvisibility=hidden",
        "-Wno-implicit-fallthrough",
    ] + OPENXR_ANDROID_COPTS,
    defines = [
        "XR_USE_PLATFORM_ANDROID=1",
        "XR_KHR_LOADER_INIT_SUPPORT=1",
    ],
    features = [
        "-use_header_modules",
    ],
    includes = [
        "generated/include",
        "generated/src/loader",
        "src/common",
        "src/loader",
    ],
    linkopts = [
        "-ldl",
        "-lm",
    ] + OPENXR_ANDROID_LINKOPTS,
    deps = [
        ":openxr_headers_androidxr",
        "//third_party/android-jni-wrappers",
        "@jsoncpp",
    ],
)
