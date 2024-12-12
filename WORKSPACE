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

workspace(name = "com_google_jxr_natives")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "bazel_skylib",
    sha256 = "f7be3474d42aae265405a592bb7da8e171919d74c16f082a5457840f06054728",
    urls = [
        "https://github.com/bazelbuild/bazel-skylib/releases/download/1.2.1/bazel-skylib-1.2.1.tar.gz",
    ],
)

http_archive(
    name = "abseil-cpp",
    patch_args = ["-p1"],
    strip_prefix = "abseil-cpp-20240722.0",
    type = ".tar.gz",
    url = "https://github.com/abseil/abseil-cpp/archive/refs/tags/20240722.0.tar.gz",
)

http_archive(
    name = "build_bazel_rules_android",
    strip_prefix = "rules_android-0.5.1",
    url = "https://github.com/bazelbuild/rules_android/archive/refs/tags/v0.5.1.zip",
)

RULES_ANDROID_NDK_COMMIT = "81ec8b79dc50ee97e336a25724fdbb28e33b8d41"

RULES_ANDROID_NDK_SHA = "b29409496439cdcdb50a8e161c4953ca78a548e16d3ee729a1b5cd719ffdacbf"

http_archive(
    name = "rules_android_ndk",
    sha256 = RULES_ANDROID_NDK_SHA,
    strip_prefix = "rules_android_ndk-%s" % RULES_ANDROID_NDK_COMMIT,
    url = "https://github.com/bazelbuild/rules_android_ndk/archive/%s.zip" % RULES_ANDROID_NDK_COMMIT,
)

load("@rules_android_ndk//:rules.bzl", "android_ndk_repository")

# Set the android api level to 29, which is the minimum required for creating Java Binders in the
# native code.
android_ndk_repository(
    name = "androidndk",
    api_level = 29,
)

register_toolchains("@androidndk//:all")

android_sdk_repository(
    name = "androidsdk",
)

## jsoncpp is required by OpenXR.
http_archive(
    name = "jsoncpp",
    sha256 = "f409856e5920c18d0c2fb85276e24ee607d2a09b5e7d5f0a371368903c275da2",
    strip_prefix = "jsoncpp-1.9.5",
    url = "https://github.com/open-source-parsers/jsoncpp/archive/refs/tags/1.9.5.tar.gz",
)
