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

package(
    default_visibility = ["//visibility:public"],
    features = ["header_modules"],
)

licenses(["notice"])

cc_library(
    name = "zstdlib",
    srcs = glob([
        "lib/common/*.c",
        "lib/common/*.h",
        "lib/compress/*.c",
        "lib/compress/*.h",
        "lib/decompress/*.c",
        "lib/decompress/*.h",
    ]),
    defines = [
      "ZSTD_MULTITHREAD=1",
      "ZSTD_DISABLE_ASM=1",
    ],
    linkopts = ["-pthread"],
    hdrs = ["lib/zstd.h", "lib/zstd_errors.h"],
    includes = [".", "lib"],
    include_prefix = "stdlib",
)
