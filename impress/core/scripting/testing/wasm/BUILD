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

load("//build_tools:imp_app.bzl", "imp_app")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

imp_app(
    name = "e2e",
    srcs = [
        "e2e_view.cc",
    ],
    platforms = [
        "wasm",
    ],
    wasm_web_server_filegroup = "//javascript/testing:test_files",
    deps = [
        "//core/scripting",
        "//core/scripting:basic_api",
        "//core/scripting:scripting_system_web",
        "//core/scripting/proto:test_cc_proto",
        "//core/view/framework/tests/data:test_model_resources",
        "@com_google_absl//absl/memory",
    ],
)
