load("@bazel_skylib//:bzl_library.bzl", "bzl_library")
load("//devtools/python/blaze:pytype.bzl", "pytype_strict_binary")

licenses(["notice"])

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

pytype_strict_binary(
    name = "compile_metal_shader",
    srcs = ["compile_metal_shader.py"],
    deps = [
        "//third_party/py/absl:app",
        "//third_party/py/absl/flags",
        "//third_party/py/absl/logging",
    ],
)

bzl_library(
    name = "postprocess_metal_shaders_bzl",
    srcs = ["postprocess_metal_shaders.bzl"],
    deps = [
        "//third_party/bazel_rules/apple_support/lib:apple_support",
        "@bazel_skylib//lib:dicts",
        "@third_party//filament:filament_bzl",
    ],
)
