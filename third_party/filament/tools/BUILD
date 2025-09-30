load(
    "@third_party//filament:filament.bzl",
    "filament_copts",
    "if_vulkan",
)
load(
    "@third_party//filament:license_helpers.bzl",
    "list_licenses",
)

licenses(["notice"])

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

list_licenses(
    name = "resgen_licenses",
    srcs = [
        "@third_party//filament:getopt_license",
    ],
    outs = ["resgen_licenses/licenses/licenses.inc"],
    delim = "FILAMENT__",
)

# Filament resource generator tool.
cc_binary(
    name = "resgen",
    srcs = glob([
        "resgen/src/*.cpp",
        "resgen/src/*.h",
    ]),
    copts = filament_copts(),
    includes = [
        "resgen/src",
        "resgen_licenses",
    ],
    deps = [
        ":resgen_licenses",
        "@third_party//filament/libs:utils",
        "@third_party//filament/libs:utils_hdrs",
    ],
)

list_licenses(
    name = "uberz_licenses",
    srcs = [
        "@third_party//filament:getopt_license",
        "@third_party//filament:zstdlib_license",
    ],
    outs = ["uberz_licenses/licenses/licenses.inc"],
    delim = "FILAMENT__",
)

# Filament ubershaders archive tool.
cc_binary(
    name = "uberz",
    srcs = glob([
        "uberz/src/*.cpp",
        "uberz/src/*.h",
    ]),
    copts = filament_copts(),
    includes = [
        "uberz/src",
        "uberz_licenses",
    ],
    deps = [
        ":uberz_licenses",
        "@robin_map",
        "@third_party//filament/libs:uberz",
        "@third_party//filament/libs:utils",
        "@third_party//filament/libs:utils_hdrs",
        "@zstdlib",
    ],
)

list_licenses(
    name = "matc_licenses",
    srcs = [
        "@third_party//filament:getopt_license",
        "@third_party//filament:glslang_license",
        "@third_party//filament:spirv_cross_license",
        "@third_party//filament:spirv_tools_license",
    ],
    outs = ["matc_licenses/licenses/licenses.inc"],
    delim = "FILAMENT__",
)

cc_library(
    name = "libmatc",
    srcs = glob(["matc/src/matc/**/*.cpp"]),
    hdrs = glob(["matc/src/matc/**/*.h"]),
    copts = filament_copts() + [
        "-Ithird_party/glslang/glslang/Include",
        "-Ithird_party/glslang/glslang/MachineIndependent",
        "-Ithird_party/glslang/glslang/Public",
        "-Ithird_party/glslang/SPIRV",
    ],
    includes = [
        "matc/src",
        "matc_licenses",
    ],
    linkstatic = 1,
    deps = [
        ":matc_licenses",
        "@glslang",
        "@glslang//:SPIRV",
        "@smol_v//:smol-v",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:filamat",
        "@third_party//filament/libs:filamat_builtinResource",
        "@third_party//filament/libs:matp",
        "@third_party//filament/libs:utils",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

# Filament material compiler library.
cc_binary(
    name = "matc",
    srcs = [
        "matc/src/main.cpp",
    ],
    copts = filament_copts(),
    linkopts = [
        "-pthread",
        "-ldl",
    ],
    deps = [
        ":libmatc",
    ],
)

cc_binary(
    name = "matinfo",
    srcs = [
        "matinfo/src/main.cpp",
    ],
    copts = filament_copts() + [
        "-Ithird_party/glslang/glslang/Include",
        "-Ithird_party/glslang/glslang/MachineIndependent",
        "-Ithird_party/glslang/glslang/Public",
        "-Ithird_party/glslang/SPIRV",
    ],
    includes = [
        "matc_licenses",
    ],
    linkopts = [
        "-pthread",
        "-ldl",
    ],
    deps = [
        ":matc_licenses",
        "@glslang",
        "@glslang//:SPIRV",
        "@smol_v//:smol-v",
        "@spirv_cross//:spirv_cross_lib",
        "@spirv_tools",
        "@spirv_tools//:spirv_tools_opt",
        "@third_party//filament/libs:filabridge",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:filaflat",
        "@third_party//filament/libs:matdbg",
        "@third_party//filament/libs:utils",
        "@third_party//filament/libs:utils_hdrs",
    ],
)

cc_binary(
    name = "matedit",
    srcs = glob([
        "matedit/src/*.cpp",
        "matedit/src/*.h",
    ]),
    copts = filament_copts(),
    includes = [
        "matedit_licenses",
    ],
    deps = [
        ":matedit_licenses",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/libs:filamat",
        "@third_party//filament/libs:matdbg",
        "@third_party//filament/libs:utils_hdrs",
    ] + if_vulkan(
        ["@third_party//filament/libs:filaflat_vulkan"],
        ["@third_party//filament/libs:filaflat"],
    ),
)

list_licenses(
    name = "matedit_licenses",
    srcs = [
        "@third_party//filament:getopt_license",
    ],
    outs = ["matedit_licenses/licenses/licenses.inc"],
    delim = "FILAMENT__",
)

list_licenses(
    name = "cmgen_licenses",
    srcs = [
        "@third_party//filament:getopt_license",
        "@third_party//filament:imath_license",
        "@third_party//filament:libpng_license",
        "@third_party//filament:libz_license",
        "@third_party//filament:openexr_license",
    ],
    outs = ["cmgen_licenses/licenses/licenses.inc"],
    delim = "FILAMENT__",
)

#Filament cube map generator library.
cc_binary(
    name = "cmgen",
    srcs = glob([
        "cmgen/src/*.cpp",
        "cmgen/src/*.h",
    ]),
    copts = filament_copts() + [
        "-Wno-sometimes-uninitialized",
    ] + ["-fexceptions"],
    includes = [
        "cmgen/src",
        "cmgen_licenses",
    ],
    linkopts = select({
        "@platforms//os:linux": [],
        "@platforms//os:android": [],
        "//conditions:default": [],
    }) + [
        "-pthread",
        "-ldl",
    ],
    deps = [
        ":cmgen_licenses",
        "@third_party//filament/libs:filabridge",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:ibl",
        "@third_party//filament/libs:ibl_hdrs",
        "@third_party//filament/libs:image",
        "@third_party//filament/libs:image_hdrs",
        "@third_party//filament/libs:imageio",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:utils",
        "@third_party//filament/libs:utils_hdrs",
    ],
)

cc_binary(
    name = "prebuilt_matc",
    srcs = [
        "matc/src/main.cpp",
    ],
    copts = filament_copts(),
    features = [
        "-pie",
        "fully_static_link",
    ],
    linkopts = [
        "-pthread",
        "-ldl",
    ],
    deps = [
        ":libmatc",
    ],
)

cc_binary(
    name = "prebuilt_cmgen",
    srcs = glob([
        "cmgen/src/*.cpp",
        "cmgen/src/*.h",
    ]),
    copts = filament_copts() + [
        "-Wno-sometimes-uninitialized",
    ] + ["-fexceptions"],
    features = [
        "-pie",
        "fully_static_link",
    ],
    includes = [
        "cmgen/src",
        "cmgen_licenses",
    ],
    linkopts = select({
        "@platforms//os:linux": [],
        "@platforms//os:android": [],
        "//conditions:default": [],
    }) + [
        "-pthread",
        "-ldl",
    ],
    deps = [
        ":cmgen_licenses",
        "@third_party//filament/libs:filabridge",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:ibl",
        "@third_party//filament/libs:ibl_hdrs",
        "@third_party//filament/libs:image",
        "@third_party//filament/libs:image_hdrs",
        "@third_party//filament/libs:imageio",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:utils",
        "@third_party//filament/libs:utils_hdrs",
    ],
)
