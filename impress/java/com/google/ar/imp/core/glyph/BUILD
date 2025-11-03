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

load("//third_party/android/androidx_test/build_extensions:kt_android_library.bzl", "kt_android_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

kt_android_library(
    name = "glyph",
    srcs = [
        "BidiRuns.kt",
        "GlyphAdvance.kt",
        "GlyphSource.kt",
        "PathGlyphSource.kt",
        "ShaperGlyphSource.kt",
        "utils.kt",
    ],
    disable_lint_checks = ["NewApi"],
    deps = [
        "//third_party/java/androidx/graphics/path",
        "@third_party//filament/android:filament_android_no_jni",
    ],
)
