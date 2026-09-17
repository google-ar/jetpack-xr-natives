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

load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "icon_font",
    hdrs = ["icon_font.h"],
    deps = ["@dear_imgui"],
    alwayslink = 1,
)

exports_files(
    srcs = [
        "GoogleSans-ArabicMedium.ttf",
        "GoogleSans-JapaneseMedium.ttf",
        "GoogleSans-Medium.ttf",
        "GoogleSans-MyanmarMedium.ttf",
        "MaterialSymbolsOutlined-Regular.ttf",
    ],
)
