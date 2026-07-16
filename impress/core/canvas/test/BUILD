# Copyright 2026 Google LLC
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

load("//build_tools:imp_app.bzl", "imp_app")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

imp_app(
    name = "canvas_test",
    srcs = [
        "canvas_test_view.cc",
    ],
    android_assets = [
        "//core/canvas/assets:canvas_scuba_test_assets",
    ],
    android_java_deps = [
        "//java/com/google/ar/imp/core/glyph",
        "//third_party/kotlin/kotlin:kotlin_stdlib",
    ],
    android_package = "com.google.ar.imp.core.canvas.test",
    ios_bundle_name = "canvasTest",
    platforms = [
        "android",
    ],
    deps = [
        "//core/canvas:canvas_interfaces",
        "//core/canvas:canvas_source",
        "//core/canvas:constants",
        "//core/canvas/assets:canvas_scuba_test_assets",
        "//core/render",
        "//core/render:primitive_shape_renderer",
        "//core/render:primitive_shape_renderer_state_cc_proto",
        "//core/view/framework:framework_proto_cc",
        "@com_google_absl//absl/strings",
    ],
)
