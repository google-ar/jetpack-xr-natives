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

load("@rules_license//rules:license_kind.bzl", "license_kind")

# Impress is a lightweight 3D app framework.
# Note: this is currently a placeholder for a migration from //third_party/impress
load(
    "//build_tools:imp.bzl",
    "if_android",
    "if_linux_or_android",
    "imp_default_jni_linkopts",
    "imp_google3_copts",
)
load("//build_tools:imp_extensions.bzl", "imp_extensions")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

exports_files([
    "LICENSE",
])

license_kind(
    name = "license",
    conditions = [
        "notice",
    ],
)

# The primary target for accessing Impress's API.
#
# Depend on 'platform_*' targets to include required dependencies
# for running on supported platforms (linux, macos, android, ios)
# Depend on 'platform_mobile' target to automatically support all mobile platforms.
# Depend on 'platform_desktop' target to automatically support all desktop platforms.
#
# Typically, do not depend on a 'platform_*' target in unit tests.
#
# See //samples/simple
# For an example of how to make an ImpView application.
#
# Provides access to all ImpView API through forwarding headers
# according to the best practices outline in option 2 here:
# (broken link)
# A "flavor" of the imp api target that excludes all optional features.
# If you depend on this target, you will need to have either
# :impl_full or :impl_minimal (but not both) somewhere in your deps
# of your final build or you will get linker errors.
# Generally, the right approach is to depend on :api everywhere and then
# add a dep to one of the :impls at the top level of your app.
cc_library(
    name = "api",
    hdrs = [
        "imp.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//core:config",
        "//core/assets",
        "//core/async:executor",
        "//core/async:future",
        "//core/async:future_status_utils",
        "//core/common",
        "//core/common:context",
        "//core/common:debug_draw",
        "//core/common:filament_helpers",
        "//core/common:holdable",
        "//core/common:invocable",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/common:resource_helpers",
        "//core/common:smooth",
        "//core/geometry/shapes",
        "//core/input",
        "//core/lighting:environment_light_factory",
        "//core/materials",
        "//core/math",
        "//core/math:aabb_helpers",
        "//core/model",
        "//core/model/mesh",
        "//core/model/mesh:mesh_data",
        "//core/ncsb",
        "//core/render",
        "//core/resources",
        "//core/scene_handles",
        "//core/scene_handles:scene_handle_status_utils",
        "//core/scene_handles:scene_handles_imp_cc_proto",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view:view_host",
        "//core/view/framework",
        "//core/view/utils:device",
        "//core/view/utils:frame_time",
        "//core/window:filament_host",
        "//core/window:window_rotation",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

# This impl "flavor" includes implementations of all optional extensions.
imp_extensions(
    name = "impl_full",
    extensions = [
        "meshopt",
        "draco",
        "verification",
        "basis",
        "behavior",
        "webp",
        "mesh_features",
    ],
)

# This impl "flavor" reduces binary size with a combination of noop and impl versions of optional
# features.
# TODO: figure out a cleaner way of supporting optional gltf features.
imp_extensions(
    name = "impl_minimal",
    extensions = [],
)

# This target automatically brings in required platform-specific
# dependencies depending on the platform being built for.
#
# See //samples/simple as an example.
cc_library(
    name = "platform_mobile",
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = if_linux_or_android(
        [":platform_android"],
        otherwise = [":platform_ios"],
    ),
    alwayslink = 1,
)

# This target automatically brings in required android-specific dependencies.
#
# See //java/com/google/ar/imp/samples/simple:simple_app
# To see how it is used to create an android binary.
cc_library(
    name = "platform_android",
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":api",
        "//core/view/platforms/android",
    ],
    alwayslink = 1,
)

# This target automatically brings in required ios-specific dependencies.
#
# See //samples/simple/simple_ios
# To see how it is used to create an ios binary.
cc_library(
    name = "platform_ios",
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":api",
        "//core/view/platforms/ios",
    ],
    alwayslink = 1,
)

# This target automatically brings in required desktop-specific dependencies.
# google-linux, and macos are both supported.
#
# See //samples/simple:simple_desktop
# To see how it is used to create a desktop binary.
cc_library(
    name = "platform_desktop",
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":api",
        "//core/view/platforms/desktop",
    ],
    alwayslink = 1,
)

# This target is for a web-assembly build of Imp that works in the browser.
cc_library(
    name = "platform_wasm",
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":api",
        "//core/view/platforms/wasm",
    ],
    alwayslink = 1,
)

cc_library(
    name = "impress_lib_xr",
    testonly = True,
    copts = imp_google3_copts(),
    linkopts = imp_default_jni_linkopts() + if_android(["-llog -static-libstdc++"]),
    deps = [
        ":api",
        ":impl_full",
        ":testing_api",
        "//core/common:debug_draw",
        "//core/lighting:environment_light",
        "//core/lighting:environment_light_factory",
        "//core/lighting:image_based_lighting_asset",
        "//core/lighting:image_based_lighting_asset_iblprefilter_loader",
        "//core/math",
        "//core/render_passes:texture_pipeline_renderer",
        "//core/scripting",
        "//core/scripting:basic_api",
        "//core/view/framework",
        "//samples/simple:simple_assets",
    ] + if_android([
        ":platform_android",
        "//core/scripting/message_handlers/android:android_view_renderer",
        "//core/scripting/message_handlers/android:android_view_request_handler",
        "//core/view/platforms/xr_android:xr",
        "//core/view/platforms/xr_android:xr_action_controller",
        "//core/view/platforms/xr_android:xr_events_proto_cc",
        "//core/view/platforms/xr_android:xr_jni",
    ]),
    alwayslink = 1,
)

# This target provides API to help test ImpView code in cc_tests.
# This includes a fixture that sets up an imp::View inside of a test.
# Note: this includes :impl_full so it is not compatible with impl_minimal.
cc_library(
    name = "testing_api",
    testonly = True,
    hdrs = ["testing.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":api",
        ":impl_full",
        "//core/common:imp_matchers",
        "//testing",
    ],
    alwayslink = 1,
)

# This target provides API to help test ImpView code in cc_tests.
# This includes a fixture that sets up an imp::View inside of a test.
# Note: this includes :impl_minimal so it is not compatible with impl_full.
cc_library(
    name = "testing_api_minimal",
    testonly = True,
    hdrs = ["testing.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":api",
        ":impl_minimal",
        "//core/common:imp_matchers",
        "//testing:testing_minimal",
    ],
    alwayslink = 1,
)

genrule(
    name = "imp_hosted_assets",
    srcs = [
        "//core/loader/data:compiled_imp_default_lite_gltf_materials.zip",
        "//core/loader/data:compiled_imp_default_gltf_materials.zip",
    ],
    outs = [
        "imp_hosted_assets_fs/compiled_imp_default_lite_gltf_materials.zip",
        "imp_hosted_assets_fs/compiled_imp_default_gltf_materials.zip",
    ],
    cmd = "cp $(SRCS) $(@D)/imp_hosted_assets_fs/",
)

# To actually generate webtreemaps you must pass --objc_generate_linkmap to the
# build invocation.

filegroup(
    name = "copybara",
    srcs = glob(["**/*.bara.sky"]),
)
