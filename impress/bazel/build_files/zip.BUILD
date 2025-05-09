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

# Copyright 2009 Google Inc.
# All rights reserved.
#
# Description:
#   A BUILD file for zip.  Provided for a hermetic environment as part of the
# buildhelpers v3 initiative.  See the wiki for more information:
#   (broken link)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-layering_check",
        "-parse_headers",
    ],
)

licenses(["notice"])

exports_files(["LICENSE"])

# NOTE: Note that many files in the third_party dir do not compile on
# unix.  If you want to compile zip for something other than linux (ie: windows)
# make a separate srcs list and refactor the list below or you'll break other
# platforms's builds.

zip_core_sources = [
    "crypt.c",
    "crypt.h",
    "crc32.c",
    "ebcdic.h",
    "fileio.c",
    "globals.c",
    "revision.h",
    "tailor.h",
    "ttyio.c",
    "ttyio.h",
    "util.c",
    "ziperr.h",
    "zipfile.c",
    "zipup.c",
    "zip.h",
    "crc32.h",
]

zip_unix_sources = [
    "unix/osdep.h",
    "unix/unix.c",
    "unix/zipup.h",
]

zip_windows_sources = [
    "win32/crc_i386.c",
    "win32/nt.c",
    "win32/nt.h",
    "win32/osdep.h",
    "win32/safe_windows.h",
    "win32/win32.c",
    # "win32/rsxntwin.h",
    "win32/win32i64.c",
    "win32/win32zip.c",
    "win32/win32zip.h",
    "win32/zipup.h",
]

zip_copts = [
    "-Ithird_party/zip",
    "-DUIDGID_NOT_16BIT",
    "-DNO_BZIP2_SUPPORT",
    "-DLARGE_FILE_SUPPORT",
    "-DUNICODE_SUPPORT",
    "-DUSE_ZLIB",
    "-DHAVE_DIRENT_H",
    "-DHAVE_TERMIOS_H",
    "-Wno-unused-variable",
] + select({
    "//conditions:default": [
        "-DUNIX",
        # Suppress this even if part of -Wall.
        "-Wno-self-assign",
        "-Wno-uninitialized",
        # Suppress differ in pointer signedness warnings.
        "-Wno-pointer-sign",
    ],
})

cc_library(
    name = "zip_lib",
    srcs = zip_core_sources + zip_unix_sources,
    hdrs = [
        "crc32.h",
        "crypt.h",
        "revision.h",
        "ttyio.h",
        "zip.h",
    ],
    copts = zip_copts,
    deps = ["@zlib"],
)

cc_binary(
    name = "zip",
    srcs = [
        "deflate.c",
        "trees.c",
        "zbz2err.c",
        "zip.c",
    ],
    copts = zip_copts,
    linkopts = [],
    deps = [":zip_lib"],
)

# Some of the utilities in here need the library source compiled with -DUTIL, so
# we make two separate libraries.
cc_library(
    name = "zip_util_lib",
    srcs = zip_core_sources + zip_unix_sources,
    hdrs = [
        "revision.h",
        "zip.h",
    ],
    copts = ["-DUTIL"] + zip_copts,
    deps = ["@zlib"],
)

cc_binary(
    name = "zipsplit",
    srcs = ["zipsplit.c"],
    copts = zip_copts,
    deps = ["zip_util_lib"],
)

cc_binary(
    name = "zipnote",
    srcs = ["zipnote.c"],
    copts = zip_copts,
    deps = ["zip_util_lib"],
)

java_binary(
    name = "CreateZipWithNEntries",
    srcs = ["CreateZipWithNEntries.java"],
    javacopts = ["-Xep:DefaultPackage:OFF"],
    main_class = "CreateZipWithNEntries",
)
