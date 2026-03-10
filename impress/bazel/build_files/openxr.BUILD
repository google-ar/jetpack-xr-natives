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

# Khronos OpenXR specification
load("@rules_license//rules:license_kind.bzl", "license_kind")
load(
    ":flags_and_opts.bzl",
    "OPENXR_ANDROID_COPTS",
    "OPENXR_ANDROID_LINKOPTS",
    "OPENXR_COPTS",
    "OPENXR_GEN_HEADER_COPTS",
)

package(
    default_applicable_licenses = [":license"],
    default_visibility = ["//visibility:public"]
)

license_kind(
    name = "license",
    conditions = [
      "notice",
    ],
)

licenses(["notice"])

exports_files(["LICENSE"])

cc_library(
    name = "openxr_headers_androidxr",
    hdrs = glob(["generated/include/openxr/*.h"]),
    includes = [
        "generated/include/openxr",
    ],
    strip_include_prefix = "//OpenXR_KHR/generated/include",
)

cc_library(
    name = "generated_openxr_headers_public",
    hdrs = glob(["generated/include/public/*.h"]),
    includes = [
        "generated/include/public",
    ],
    strip_include_prefix = "//OpenXR_KHR/generated/include",
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
    ]) + select({
        "@com_google_impress//core:android": glob([
            "src/external/android-jni-wrappers/wrap/**",
            "src/external/jnipp/*.cpp",
            "src/external/jnipp/*.h",
        ]),
        "//conditions:default": [],
    }),
    copts = [
        "-DAPI_NAME=\"OpenXR\"",
        "-DDISABLE_STD_FILESYSTEM",
        "-DXRAPI_DLL_EXPORT",
        "-Dopenxr_loader_EXPORTS",
        "-DNDEBUG",
        "-fexceptions",
        "-fvisibility=hidden",
        "-Wno-implicit-fallthrough",
    ] + select({
        "@com_google_impress//core:android": OPENXR_ANDROID_COPTS,
        "//conditions:default": [
            "-DXR_OS_LINUX",
        ],
    }),
    defines = select({
        "@com_google_impress//core:android": [
            "XR_USE_PLATFORM_ANDROID=1",
            "XR_KHR_LOADER_INIT_SUPPORT=1",
        ],
        "//conditions:default": [
        ],
    }),
    features = select({
        "@com_google_impress//core:android": [],
        "//conditions:default": [
            "-use_header_modules",
        ],
    }),
    includes = [
        "generated/include",
        "generated/src/loader",
        "src/common",
        "src/loader",
    ] + select({
        "@com_google_impress//core:android": [
            "src/external/android-jni-wrappers",
            "src/external/jnipp",
        ],
        "//conditions:default": [],
    }),
    linkopts = [
        "-ldl",
        "-lm",
    ] + select({
        "@com_google_impress//core:android": OPENXR_ANDROID_LINKOPTS,
        "//conditions:default": [],
    }),
    deps = [
        ":openxr_headers_androidxr",
        "@jsoncpp",
    ],
)
