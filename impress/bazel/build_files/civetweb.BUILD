# Copyright 2025 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Description:
# CivetWeb provides easy to use, powerful, C (C/C++) embeddable web server with optional CGI, SSL
# and Lua support.

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

exports_files(["LICENSE"])

cc_library(
    name = "civetserver",
    hdrs = ["include/CivetServer.h"],
    copts = [
        "-Wno-error=frame-larger-than=",
    ],
    includes = ["include"],
    deps = [
        ":civetserver_lib",
    ],
)

cc_library(
    name = "civetserver_lib",
    srcs = [
        "src/CivetServer.cpp",
    ],
    hdrs = [
        "include/CivetServer.h",
        "include/civetweb.h",
    ],
    copts = [
        "-fexceptions",
        "-Wno-error=frame-larger-than=",
    ],
    defines = [
        "MG_LEGACY_INTERFACE",
        "USE_WEBSOCKET",
    ],
    features = ["-use_header_modules"],  # Incompatible with -fexception.
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        ":civetweb_lib",
    ],
)

cc_library(
    name = "civetweb_lib",
    srcs = [
        "src/civetweb.c",
    ],
    hdrs = [
        "include/civetweb.h",
    ],
    copts = [
        "-Wno-error=frame-larger-than=",
    ],
    defines = [
        "MG_LEGACY_INTERFACE",
        "USE_WEBSOCKET",
    ],
    includes = ["include"],
    textual_hdrs = [
        "src/handle_form.inl",
        "src/md5.inl",
        "src/sha1.inl",
    ],
    visibility = ["//visibility:public"],
)
