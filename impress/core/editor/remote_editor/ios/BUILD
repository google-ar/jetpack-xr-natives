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

load("@rules_cc//cc:objc_library.bzl", "objc_library")
load(
    "//build_tools:imp.bzl",
    "imp_google3_copts",
)

package(default_visibility = ["//visibility:public"])

objc_library(
    name = "ios_web_ui_remote_editor",
    srcs = ["ios_web_ui_remote_editor.mm"],
    hdrs = ["ios_web_ui_remote_editor.h"],
    copts = imp_google3_copts(),
    data = [":ui_stream_assets_ios"],
    deps = [
        "//core:config",
        "//core/editor/remote_editor:remote_editor_base",
        "//core/math",
        "//core/ncsb",
        "//core/scripting",
        "//core/scripting:message_helpers",
        "//core/scripting/proto:bridge_cc_proto",
        "//core/video:video_writer_ios",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/scripting",
        "//core/window:filament_host",
        "@com_google_absl//absl/status:statusor",
        "@com_google_impress//core/common:log",
    ],
)

genrule(
    name = "ui_stream_html_gen",
    srcs = ["//javascript/core/editor/remote_editor/ui_stream:assets"],
    outs = ["ui_stream.html"],
    cmd = """
        for f in $(SRCS); do
          if [[ $$f == *"index.html" ]]; then
            cp $$f $@
          fi
        done
    """,
)

objc_library(
    name = "ui_stream_assets_ios",
    data = [
        ":ui_stream_html_gen",
        "//javascript/core/editor/remote_editor/ui_stream:assets/ui_stream/style.css",
        "//javascript/core/editor/remote_editor/ui_stream:assets/ui_stream/ui_stream_app.js",
    ],
)
