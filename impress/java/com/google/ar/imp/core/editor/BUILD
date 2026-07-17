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

load("@build_bazel_rules_android//android:rules.bzl", "android_library")

package(default_visibility = ["//visibility:public"])

android_library(
    name = "remote_editor",
    srcs = [
        "RemoteEditor.java",
        "RemoteEditorHttpServer.java",
        "RemoteEditorScriptApiBridge.java",
        "RemoteEditorVideoStreamer.java",
        "RemoteEditorWebSocketServer.java",
    ],
    deps = [
        "//third_party/java/WebSocket:android",
        "@maven//:androidx_annotation_annotation",
        "@maven//:com_google_guava_guava",
        "@third_party//filament/android:filament_android_no_jni",
    ],
)

android_library(
    name = "remote_editor_ui_assets",
    assets = ["//javascript/core/editor/remote_editor/ui_stream:assets"],
    assets_dir = "assets",
)
