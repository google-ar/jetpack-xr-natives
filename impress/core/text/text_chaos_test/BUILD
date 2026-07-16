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
    name = "text_chaos_test",
    srcs = [
        "text_chaos_test_view.cc",
        "text_chaos_test_view.h",
    ],
    android_activity_name = "TextChaosActivity",
    android_base_activity = "//javatests/com/google/ar/imp/text/textchaostest/test:text_chaos_activity_lib",
    android_java_deps = [
        "//java/com/google/ar/imp/core/glyph",  # unuseddeps: keep
    ],
    android_package = "com.google.ar.imp.text.textchaostest",
    platforms = [
        "desktop",
        "android",
    ],
    deps = [
        "//core/async:future",
        "//core/math",
        "//core/ncsb",
        "//core/text:text_collider",
        "//core/text:text_renderer",
        "//core/text:text_renderer_proto_cc",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "//core/view/utils:view_config_proto_cc_imp",
        "//samples/text_stress_test:dictionaries",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
    ],
)
