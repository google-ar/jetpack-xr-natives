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

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

config_setting(
    name = "arm_v8",
    values = {"android_cpu": "arm64-v8a"},
)

# Allows Impress to use OpenXR
# OpenXR is a public API for VR and AR devices.
cc_library(
    name = "openxr_loader_lib",
    deps = [
        "@third_party//OpenXR_KHR:openxr_loader",
    ],
    alwayslink = 1,
)

cc_library(
    name = "openxr_events",
    srcs = [
    ],
    hdrs = [
        "openxr_events.h",
    ],
    deps = [
        "//core/math",
        "//core/ncsb",
    ],
    alwayslink = 1,
)
