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

"""Helper functions when building the Impress Framework."""

load(
    "@third_party//filament:filament.bzl",
    "by_backend",
)

# If adding a project to Impress dependencies consider also:
# Reading through docs at (broken link)
# Adding a presubmit/check_test in third_party/impress/METADATA
# Joining the partner email list at (broken link)
# Subscribing to YAQS at (broken link)
IMP_DEPENDEES = ["//..."]

IMP_FLATBUFFER_INCLUDE_PATHS = [
    "./",
    "$(GENDIR)",
    "$(BINDIR)",
    "external/com_google_impress+",  # Used when Impress is external library.
    "external/third_party+",  # Used when Split Engine is external library.
]

IMP_FLATC_ARGS = [
    # Sceneform is c++11/14; use scoped enums.
    "--scoped-enums",
    "--no-union-value-namespacing",
    # Use the full include path for generated includes.
    "--keep-prefix",
]

# Sanitize a dependency so that it works correctly from code that includes
# Sceneform as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def if_apple(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:macos"): a,
        clean_dep("@com_google_impress//core:ios"): a,
        "//conditions:default": otherwise,
    })

def if_non_apple(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:macos"): otherwise,
        clean_dep("@com_google_impress//core:ios"): otherwise,
        "//conditions:default": a,
    })

def if_linux_or_android(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:linux"): a,
        clean_dep("@com_google_impress//core:android"): a,
        "//conditions:default": otherwise,
    })

def if_android(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:android"): a,
        "//conditions:default": otherwise,
    })

def if_android_min_sdk_at_least_26(a, otherwise = []):
    return a

def if_non_android(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:android"): otherwise,
        "//conditions:default": a,
    })

def if_macos(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:macos"): a,
        "//conditions:default": otherwise,
    })

def if_ios(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:ios"): a,
        "//conditions:default": otherwise,
    })

def if_windows(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:windows"): a,
        "//conditions:default": otherwise,
    })

def if_linux(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:linux"): a,
        "//conditions:default": otherwise,
    })

def if_desktop(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:linux"): a,
        clean_dep("@com_google_impress//core:macos"): a,
        clean_dep("@com_google_impress//core:windows"): a,
        "//conditions:default": otherwise,
    })

def if_mobile(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:android"): a,
        clean_dep("@com_google_impress//core:ios"): a,
        "//conditions:default": otherwise,
    })

def if_mobile_arm64(a, otherwise = []):
    return select({
        "@third_party//filament:android_arm64": a,
        "@third_party//filament:ios_arm64": a,
        "//conditions:default": otherwise,
    })

def if_emscripten(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:wasm"): a,
        "//conditions:default": otherwise,
    })

def if_linux_google_prod(a, otherwise = []):
    return select({
        clean_dep("@platforms//os:linux"): a,
        "//conditions:default": otherwise,
    })

def if_remote_desktop(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:remote_desktop"): a,
        "//conditions:default": otherwise,
    })

def if_dev_runtime(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:impel_dev_runtime"): a,
        clean_dep("@com_google_impress//core:imp_dev_runtime"): a,
        "//conditions:default": otherwise,
    })

def if_optimized(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:opt"): a,
        "//conditions:default": otherwise,
    })

def if_native_debug_enabled(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:native_debug"): a,
        "//conditions:default": otherwise,
    })

def if_imp_embed_assets(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//build_tools:imp_embed_assets"): a,
        "//conditions:default": otherwise,
    })

def if_enable_face_tracking(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:imp_enable_face_tracking"): a,
        "//conditions:default": otherwise,
    })

def if_enable_editor_on_startup(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:imp_enable_editor_on_startup"): a,
        "//conditions:default": otherwise,
    })

def if_invert_editor_input(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:imp_invert_editor_input"): a,
        "//conditions:default": otherwise,
    })

def if_include_stereo_variant_by_default(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:imp_include_stereo_variant_by_default"): a,
        "//conditions:default": otherwise,
    })

def if_enable_stereo_type_multiview(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:imp_enable_stereo_type_multiview"): a,
        "//conditions:default": otherwise,
    })

def if_android_external_texture_surface_uses_image_reader(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core:imp_android_external_texture_surface_uses_image_reader"): a,
        "//conditions:default": otherwise,
    })

def if_enable_recipe_experimental(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core/recipes:imp_enable_recipe_experimental"): a,
        "//conditions:default": otherwise,
    })

def if_imp_use_local_split_engine_materials(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core/split_engine/materials:imp_use_local_split_engine_materials"): a,
        "//conditions:default": otherwise,
    })

def if_imp_perfetto_enabled(enabled, disabled):
    return select({
        clean_dep("@com_google_impress//core:imp_perfetto_enabled"): enabled,
        "//conditions:default": disabled,
    })

def if_imp_disable_future_validation(enabled, disabled):
    return select({
        clean_dep("@com_google_impress//core:imp_disable_future_validation"): enabled,
        "//conditions:default": disabled,
    })

def if_imp_split_engine_allow_experimental_apis(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core/split_engine:imp_split_engine_allow_experimental_apis"): a,
        "//conditions:default": otherwise,
    })

def if_imp_profiler_enable_memory_graph(a, otherwise = []):
    return select({
        clean_dep("@com_google_impress//core/performance:imp_profiler_enable_memory_graph"): a,
        "//conditions:default": otherwise,
    })

# Whether the target we're compiling for should pack-in generic materials for glTF loading.
# We pack-in on desktop platforms, on the iOS Simulator (which can't hit gstatic), or if requested
# on the command line via --define=IMP_EMBED_ASSETS=1
def imp_embed_generic_materials():
    # Forge-on-mac (e.g. for Linux-hosted iOS tests) runs via VMWare, so no Metal and no gstatic.
    return select({
        "@com_google_impress//build_tools:should_embed_generic_materials": True,
        "//conditions:default": False,
    })

def imp_embed_shared_assets():
    return imp_embed_generic_materials()

# A looser set of copts which is compatible with e.g. //util/status
def imp_google3_copts():
    out_copts = [
        # Match google3/third_party disabled warnings.
        "-Wno-unused-local-typedef",
        "-Wno-missing-braces",
        "-Wno-macro-redefined",
        "-Wno-c++20-designator",
        # We don't use exceptions.
        "-fno-exceptions",
    ] + if_optimized([], otherwise = [
        "-g",
        # Optimize for small code size in non-opt builds.
        # In opt builds, this flag actually makes the size larger.
        # However, we still care about code-size in non-opt builds.
        "-Os",
    ])
    return out_copts

def imp_copts():
    out_copts = imp_google3_copts() + if_emscripten([], otherwise = [
        # We don't use runtime type identification.  We can't define this everywhere, however,
        # for example any unit tests using gunit require rtti in order to compile.
        # rtti should not be disabled for emscripten
        "-fno-rtti",
    ]) + if_mobile_arm64([
        # See: (broken link)
        # Filament is designed to run with ffast-math, because it generates significantly
        # smaller and faster floating point assembly.  Impress defines it as well, since
        # it makes extensive use of filament's math library.  Be warned that, for example,
        # isnan is a no-op, and +0/-0 are equivalent.  We cannot define this universally,
        # because e.g. //util/task require errno in math functions to be defined for PCH
        # compatibility.
        "-ffast-math",
    ], otherwise = [])
    return out_copts

def imp_test_copts():
    out_copts = imp_google3_copts()
    return out_copts

def imp_defines():
    out_defines = [
    ] + by_backend(
        metal = ["IMP_MATERIAL_API_CONFIG=METAL"],
        opengl = ["IMP_MATERIAL_API_CONFIG=OPENGL"],
        vulkan = ["IMP_MATERIAL_API_CONFIG=VULKAN"],
        gl_vulkan = ["IMP_MATERIAL_API_CONFIG=GL_VULKAN"],
    ) + if_optimized(
        ["NDEBUG"],
    ) + if_dev_runtime(
        ["IMP_RUNTIME_CONFIG=DEV"],
        otherwise = ["IMP_RUNTIME_CONFIG=SHIP"],
    ) + if_enable_face_tracking(
        ["IMP_ENABLE_FACE_TRACKING"],
    ) + if_enable_editor_on_startup(
        ["IMP_ENABLE_EDITOR_ON_STARTUP"],
    ) + if_invert_editor_input(
        ["IMP_INVERT_EDITOR_INPUT"],
    ) + if_include_stereo_variant_by_default(
        ["IMP_INCLUDE_STEREO_VARIANT_BY_DEFAULT"],
    ) + if_enable_stereo_type_multiview(
        ["IMP_ENABLE_STEREO_TYPE_MULTIVIEW"],
    ) + if_android_external_texture_surface_uses_image_reader(
        ["IMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER"],
    ) + if_enable_recipe_experimental(
        ["IMP_ENABLE_RECIPE_EXPERIMENTAL"],
    ) + if_imp_use_local_split_engine_materials(
        ["IMP_USE_LOCAL_SPLIT_ENGINE_MATERIALS"],
    ) + if_imp_perfetto_enabled(
        enabled = ["IMP_TRACE_USE_PERFETTO=1"],
        disabled = ["IMP_TRACE_USE_PERFETTO=0"],
    ) + if_imp_disable_future_validation(
        enabled = ["IMP_DISABLE_FUTURE_VALIDATION=1"],
        disabled = ["IMP_DISABLE_FUTURE_VALIDATION=0"],
    ) + if_imp_split_engine_allow_experimental_apis(
        ["IMP_SPLIT_ENGINE_ALLOW_EXPERIMENTAL_APIS"],
    ) + if_imp_profiler_enable_memory_graph(
        ["IMP_PROFILER_MEMORY_GRAPH=1"],
    )
    return out_defines

def imp_linkopts():
    out_linkopts = [
    ] + [] + if_apple(
        [],
        otherwise = [
            # impress:insert(OSS) "-Wl,--gc-sections",
        ],
    )
    return out_linkopts

def imp_malloc_wrap_linkopts():
    return [
        "-Wl,--wrap=malloc",
        "-Wl,--wrap=free",
        "-Wl,--wrap=calloc",
        "-Wl,--wrap=realloc",
        "-Wl,--wrap=aligned_alloc",
        "-Wl,--wrap=posix_memalign",
        "-Wl,--wrap=memalign",
    ]

def imp_default_jni_linkopts():
    return if_android([
        "-lGLESv3",
        "-lEGL",
        "-landroid",
        # impress:insert(OSS) "-Wl,--no-undefined",
    ]) + if_android_min_sdk_at_least_26([
        # The native window library is only available on API level 26 and above.
        "-lnativewindow",
    ])

def imp_default_jni_binary_name():
    # LINT.IfChange
    return "libimp_view_jni.so"
    # LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/view/View.java)

# Returns the default name of the .so loaded by ImpXrRenderer.java when using Impress in XR.
#
# This is different from imp_default_jni_binary() to prevent imp_app.bzl from generating duplicate
# .so targets. Builds that don't use imp_app can use this name or pass in a custom .so name using
# SetupParams.
def imp_default_xr_jni_binary_name():
    # LINT.IfChange
    return "libimp_view_xr_jni.so"
    # LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/view/xr/ImpXrApi.java)

# Returns the default name of the .so loaded by ImpSplitEngineRenderer.java when using Impress for
# Split Engine mode.
#
# This is different from imp_default_jni_binary() to prevent imp_app.bzl from generating duplicate
# .so targets. Builds that don't use imp_app can use this name or pass in a custom .so name using
# SetupParams.
def imp_default_split_engine_jni_binary_name():
    # LINT.IfChange
    return "libimp_view_split_engine_jni.so"
    # LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/view/splitengine/ImpSplitEngineApi.java)
