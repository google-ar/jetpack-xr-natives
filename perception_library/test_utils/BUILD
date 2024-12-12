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

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_binary(
    name = "libfake_perception_library_jni.so",
    testonly = 1,
    srcs = ["fake_perception_library_jni.cc"],
    linkopts = [
        "-landroid",
        "-lbinder_ndk",
    ],
    linkshared = 1,
    deps = [
        ":fake_perception_library_state",
        "//third_party/OpenXR_KHR:openxr_headers_androidxr",
        "@abseil-cpp//absl/base",
        "//common:jni_utils",
        "//perception_library:jni_utils",
    ],
)

cc_library(
    name = "fake_perception_library_state",
    testonly = 1,
    hdrs = ["fake_perception_library_state.h"],
    deps = [
        "//third_party/OpenXR_KHR:openxr_headers_androidxr",
        "@abseil-cpp//absl/base:no_destructor",
        "@abseil-cpp//absl/container:flat_hash_map",
    ],
)
