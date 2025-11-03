# Consumers of Filament.
#
# For now, Filament has restricted visibility in order to track which projects need
# to be kept green when taking new drops from Filament's github repo.  Please feel
# free to send a CL and add your project to this list if you would like to depend on
# Filament in google3.

load("@bazel_skylib//:bzl_library.bzl", "bzl_library")
load("@rules_license//rules:license_kind.bzl", "license_kind")
load("@third_party//filament:license_helpers.bzl", "describe_license", "describe_license_explicit")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

license_kind(
    name = "license",
    conditions = [
        "notice",
    ],
)

exports_files(["LICENSE"])

#define FILAMENT_PREBUILT = 0

config_setting(
    name = "from_prebuilt",
    values = {
        "define": "FILAMENT_PREBUILT=true",
    },
)

config_setting(
    name = "linux",
    constraint_values = [
        "@platforms//os:linux",
    ],
)

config_setting(
    name = "linux_aarch64",
    constraint_values = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
)

config_setting(
    name = "linux_x86_64",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
)

config_setting(
    name = "macos_aarch64",
    constraint_values = [
        "@platforms//cpu:aarch64",
        "@platforms//os:macos",
    ],
)

config_setting(
    name = "macos_x86_64",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:macos",
    ],
)

config_setting(
    name = "macos",
    constraint_values = [
        "@platforms//os:macos",
    ],
)

config_setting(
    name = "ios",
    constraint_values = [
        "@platforms//os:ios",
    ],
)

config_setting(
    name = "ios_x86_64_linux",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:ios",
    ],
)

config_setting(
    name = "ios_x86_64_macos",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:ios",
    ],
    values = {
        "host_cpu": "darwin_x86_64",
    },
)

config_setting(
    name = "ios_arm64",
    constraint_values = [
        "@platforms//cpu:aarch64",
        "@platforms//os:ios",
    ],
)

config_setting(
    name = "filament_uses_metal_ios",
    constraint_values = [
        "@platforms//os:ios",
    ],
    define_values = {
        "FILAMENT_USES_METAL": "1",
    },
)

config_setting(
    name = "filament_uses_opengl_ios",
    constraint_values = [
        "@platforms//os:ios",
    ],
    define_values = {
        "FILAMENT_USES_OPENGL": "1",
    },
)

config_setting(
    name = "filament_uses_metal_ios_x86_64",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:ios",
    ],
    define_values = {
        "FILAMENT_USES_METAL": "1",
    },
)

config_setting(
    name = "filament_uses_opengl_ios_x86_64",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:ios",
    ],
    define_values = {
        "FILAMENT_USES_OPENGL": "1",
    },
    values = {
        "host_cpu": "k8",
    },
)

config_setting(
    name = "filament_uses_metal_linux",
    constraint_values = [
        "@platforms//os:linux",
    ],
    define_values = {
        "FILAMENT_USES_METAL": "1",
    },
)

config_setting(
    name = "filament_uses_metal_linux_x86_64",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    define_values = {
        "FILAMENT_USES_METAL": "1",
    },
)

config_setting(
    name = "filament_uses_opengl_linux",
    constraint_values = [
        "@platforms//os:linux",
    ],
    define_values = {
        "FILAMENT_USES_OPENGL": "1",
    },
)

config_setting(
    name = "filament_uses_opengl_linux_x86_64",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    define_values = {
        "FILAMENT_USES_OPENGL": "1",
    },
)

config_setting(
    name = "filament_uses_vulkan_linux",
    constraint_values = [
        "@platforms//os:linux",
    ],
    define_values = {
        "FILAMENT_USES_VULKAN": "1",
    },
)

config_setting(
    name = "filament_uses_vulkan_linux_x86_64",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    define_values = {
        "FILAMENT_USES_VULKAN": "1",
    },
)

# This seems wrong - nothing here is Vulkan-specific.
config_setting(
    name = "filament_uses_vulkan",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
)

config_setting(
    name = "windows",
    constraint_values = [
        "@platforms//os:windows",
    ],
)

config_setting(
    name = "android",
    constraint_values = [
        "@platforms//os:android",
    ],
)

config_setting(
    name = "android_x86",
    constraint_values = [
        "@platforms//cpu:x86_32",
        "@platforms//os:android",
    ],
)

config_setting(
    name = "android_x86_64",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:android",
    ],
)

config_setting(
    name = "android_arm",
    constraint_values = [
        "@platforms//cpu:armv7",
        "@platforms//os:android",
    ],
)

config_setting(
    name = "android_arm64",
    constraint_values = [
        "@platforms//cpu:aarch64",
        "@platforms//os:android",
    ],
)

# This shouldn't be necessary since it is the default on android, kept for backwards compatibility.
# TODO: Refactor all these rules.
config_setting(
    name = "filament_uses_gles2_android",
    constraint_values = [
        "@platforms//os:android",
    ],
    define_values = {
        "FILAMENT_USES_OPENGL_GLES2": "1",
    },
)

# TODO: Refactor all these rules.
config_setting(
    name = "filament_uses_gles3_android",
    constraint_values = [
        "@platforms//os:android",
    ],
    define_values = {
        "FILAMENT_USES_OPENGL_GLES3": "1",
    },
)

config_setting(
    name = "filament_uses_vulkan_android",
    constraint_values = [
        "@platforms//os:android",
    ],
    define_values = {
        "FILAMENT_USES_VULKAN": "1",
    },
)

config_setting(
    name = "wasm",
    constraint_values = [
        "@platforms//cpu:wasm32",
    ],
)

config_setting(
    name = "opt",
    values = {"compilation_mode": "opt"},
)

config_setting(
    name = "release",
    values = {"compilation_mode": "opt"},
)

# Used by google3/thirdparty/filament/filament.bzl to drop incompatible copt flags when bitcode is
# enabled.
config_setting(
    name = "bitcode_and_optimized",
    values = {
        "compilation_mode": "opt",
        "copt": "-fembed-bitcode",
    },
)

config_setting(
    name = "matdbg_enabled",
    define_values = {
        "MATDBG": "1",
    },
)

config_setting(
    name = "fgviewer_enabled",
    define_values = {
        "FGVIEWER": "1",
    },
)

config_setting(
    name = "systrace_enabled",
    define_values = {
        "SYSTRACE": "1",
    },
)

# Config setting to use Perfetto (instead of Atrace) for tracing on Android.
config_setting(
    name = "android_perfetto_enabled",
    constraint_values = [
        "@platforms//os:android",
    ],
    define_values = {
        "FILAMENT_PERFETTO": "1",
    },
)

config_setting(
    name = "multiview_enabled",
    define_values = {
        "FILAMENT_ENABLE_MULTIVIEW": "1",
    },
)

config_setting(
    name = "profiling_mode_enabled",
    define_values = {
        "FILAMENT_FORCE_PROFILING_MODE": "1",
    },
)

# Licenses for 3p projects used by filament
[describe_license(name = implicit) for implicit in [
    "glslang",
    "spirv_cross",
    "spirv_tools",
    "imath",
    "openexr",
]]

[describe_license_explicit(
    name = name,
    srcs = [src],
) for name, src in [
    ("getopt", "//licenses:getopt.LICENSE"),
    ("libz", "//licenses:zlib.LICENSE"),
    ("libpng", "//licenses:png.LICENSE"),
    ("zstdlib", "//licenses:zstdlib.LICENSE"),
    ("tinyexr", "//licenses:tinyexr.LICENSE"),
]]

filegroup(
    name = "shader_srcs",
    srcs = glob([
        "shaders/src/*.cs",
        "shaders/src/*.vs",
        "shaders/src/*.fs",
        "shaders/src/*.glsl",
    ]),
)

alias(
    name = "matc",
    actual = "@third_party//filament/tools:matc",
)

alias(
    name = "matinfo",
    actual = "@third_party//filament/tools:matinfo",
)

alias(
    name = "matedit",
    actual = "@third_party//filament/tools:matedit",
)

alias(
    name = "cmgen",
    actual = "@third_party//filament/tools:cmgen",
)

alias(
    name = "resgen",
    actual = "@third_party//filament/tools:resgen",
)

alias(
    name = "filament",
    actual = select({
        ":filament_uses_vulkan_android": "@third_party//filament/filament:filament_vulkan_android",
        ":filament_uses_gles2_android": "@third_party//filament/filament:filament_opengl_android_feature_level_0",
        ":filament_uses_gles3_android": "@third_party//filament/filament:filament_opengl_mobile",
        ":android": "@third_party//filament/filament:filament_opengl_android_feature_level_0",
        ":filament_uses_metal_ios": "@third_party//filament/filament:filament_metal",
        ":filament_uses_opengl_ios": "@third_party//filament/filament:filament_opengl_mobile",
        ":ios": "@third_party//filament/filament:filament_metal",
        # If building in Exoblaze, it's NextCode. Otherwise, it's the simulator.
        # Support both NextCode and iOS Simulator Tests without depending on a config flag
        # by switching based on Exoblaze.
        # TODO: Find a better way to do this.
        ":ios_x86_64_linux": "@third_party//filament/filament:filament_opengl_mobile",
        ":ios_x86_64_macos": "@third_party//filament/filament:filament_metal",
        ":filament_uses_metal_ios_x86_64": "@third_party//filament/filament:filament_metal",
        ":filament_uses_opengl_ios_x86_64": "@third_party//filament/filament:filament_opengl_mobile",
        ":wasm": "@third_party//filament/filament:filament_gles3",
        ":filament_uses_opengl_linux": "@third_party//filament/filament:filament_opengl_desktop",
        ":filament_uses_vulkan_linux": "@third_party//filament/filament:filament_vulkan_for_linux",
        ":macos": "@third_party//filament/filament:filament_opengl_desktop",
        "//conditions:default": "@third_party//filament/filament:filament_opengl_desktop",
    }),
)

bzl_library(
    name = "license_helpers_bzl",
    srcs = ["license_helpers.bzl"],
    deps = ["@rules_cc//cc:core_rules"],
)

bzl_library(
    name = "shader_helpers_bzl",
    srcs = ["shader_helpers.bzl"],
    deps = ["@rules_cc//cc:core_rules"],
)

bzl_library(
    name = "filament_bzl",
    srcs = ["filament.bzl"],
)
