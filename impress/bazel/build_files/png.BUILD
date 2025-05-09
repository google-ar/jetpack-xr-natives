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
#   libpng is the official PNG reference library.

licenses(["notice"])  # BSD/MIT-like license

exports_files(["LICENSE"])

cc_library(
    name = "png",
    srcs = [
        "png.c",
        "pngdebug.h",
        "pngerror.c",
        "pngget.c",
        "pnginfo.h",
        "pnglibconf.h",
        "pngmem.c",
        "pngpread.c",
        "pngpriv.h",
        "pngread.c",
        "pngrio.c",
        "pngrtran.c",
        "pngrutil.c",
        "pngset.c",
        "pngstruct.h",
        "pngtrans.c",
        "pngwio.c",
        "pngwrite.c",
        "pngwtran.c",
        "pngwutil.c",
    ],
    hdrs = [
        "png.h",
        "pngconf.h",
    ],
    includes = ["."],
    linkopts = ["-lm"],
    defines = ["PNG_ARM_NEON_OPT=0"],
    visibility = ["//visibility:public"],
    deps = ["@zlib"],
    # TODO: why is this necessary? "No file or dir" otherwise...
    include_prefix = "png/",
)

genrule(
    name = "snappy_stubs_public_h",
    srcs = ["scripts/pnglibconf.h.prebuilt"],
    outs = ["pnglibconf.h"],
    cmd = "sed -e 's/PNG_ZLIB_VERNUM 0/PNG_ZLIB_VERNUM 0x12d0/' $< >$@",
)

config_setting(
    name = "windows",
    values = {"cpu": "x64_windows"},
)
