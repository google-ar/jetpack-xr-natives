"""Helper functions for building Filament"""

filament_dependees = ["//:__subpackages__"]

# Sanitize a dependency so that it works correctly from code that includes
# Filament as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def if_apple(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:macos"): a,
        clean_dep("@third_party//filament:ios"): a,
        "//conditions:default": otherwise,
    })

def if_non_apple(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:macos"): otherwise,
        clean_dep("@third_party//filament:ios"): otherwise,
        "//conditions:default": a,
    })

def if_linux_or_android(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:linux"): a,
        clean_dep("@third_party//filament:android"): a,
        "//conditions:default": otherwise,
    })

def if_android(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:android"): a,
        "//conditions:default": otherwise,
    })

def if_android_min_sdk_at_least_26(a, otherwise = []):
    return a

def if_macos(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:macos"): a,
        "//conditions:default": otherwise,
    })

def if_macos_aarch64(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:macos_aarch64"): a,
        "//conditions:default": otherwise,
    })

def if_macos_x86_64(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:macos_x86_64"): a,
        "//conditions:default": otherwise,
    })

def if_ios(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:ios"): a,
        "//conditions:default": otherwise,
    })

def if_ios_simulator(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:ios_x86_64_linux"): a,
        clean_dep("@third_party//filament:ios_x86_64_macos"): a,
        
        "//conditions:default": otherwise,
    })

def if_windows(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:windows"): a,
        "//conditions:default": otherwise,
    })

def if_wasm(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:wasm"): a,
        "//conditions:default": otherwise,
    })

def if_linux(a):
    return select({
        clean_dep("@third_party//filament:linux"): a,
        "//conditions:default": [],
    })

def if_linux_aarch64(a):
    return select({
        clean_dep("@third_party//filament:linux_aarch64"): a,
        "//conditions:default": [],
    })

def if_linux_x86_64(a):
    return select({
        clean_dep("@third_party//filament:linux_x86_64"): a,
        "//conditions:default": [],
    })

def if_mobile_materials(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:wasm"): a,
        clean_dep("@third_party//filament:android"): a,
        clean_dep("@third_party//filament:ios"): a,
        "//conditions:default": otherwise,
    })

def if_desktop(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:linux"): a,
        clean_dep("@third_party//filament:macos"): a,
        "//conditions:default": otherwise,
    })

def if_matdbg(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:matdbg_enabled"): a,
        "//conditions:default": otherwise,
    })

def if_fgviewer(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:fgviewer_enabled"): a,
        "//conditions:default": otherwise,
    })

def if_systrace(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:systrace_enabled"): a,
        "//conditions:default": otherwise,
    })

def if_android_perfetto(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:android_perfetto_enabled"): a,
        "//conditions:default": otherwise,
    })

def if_multiview(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:multiview_enabled"): a,
        "//conditions:default": otherwise,
    })

def if_profiling_mode(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:profiling_mode_enabled"): a,
        "//conditions:default": otherwise,
    })

# Select labels based on which backend filament is being built with (i.e. opengl, metal, or vulkan).
#
# On Android, default is opengl.
# Use the flag FILAMENT_USES_VULKAN to switch to vulkan.
#
# On iOS default is metal when run under exoblaze, opengl when run on linux.
# (Forge-on-Mac uses a VMWare instance to host the simulator, and VMWare does not support Metal).
# Use the flag FILAMENT_USES_OPENGL to explicitly select opengl.
# Use the flag FILAMENT_USES_METAL to explicitly select metal.
#
# On MacOs, default is opengl.
# On Linux, default is vulkan.
#
# gl_vulkan is a special case that selects both opengl and vulkan. It's set to None for backwards
# compatibility to avoid breaking existing users.
def by_backend(opengl = [], metal = [], vulkan = [], gl_vulkan = None):
    """Selects based on backend, with special handling for gl_vulkan on Android."""

    # If gl_vulkan is not specified, default to opengl.
    if gl_vulkan == None:
        gl_vulkan = opengl

    return select({
        clean_dep("@third_party//filament:filament_uses_gles3_android"): opengl,
        clean_dep("@third_party//filament:filament_uses_gles2_android"): opengl,
        clean_dep("@third_party//filament:android"): opengl,
        clean_dep("@third_party//filament:filament_uses_vulkan_android"): vulkan,
        clean_dep("@third_party//filament:filament_uses_gl_vulkan_android"): gl_vulkan,
        clean_dep("@third_party//filament:filament_uses_metal_ios"): metal,
        clean_dep("@third_party//filament:filament_uses_opengl_ios"): opengl,
        clean_dep("@third_party//filament:filament_uses_vulkan_linux"): vulkan,
        clean_dep("@third_party//filament:filament_uses_metal_linux"): metal,
        clean_dep("@third_party//filament:filament_uses_opengl_linux"): opengl,
        clean_dep("@third_party//filament:ios"): metal,
        clean_dep("@third_party//filament:ios_x86_64_linux"): opengl,
        clean_dep("@third_party//filament:ios_x86_64_macos"): metal,
        clean_dep("@third_party//filament:filament_uses_metal_ios_x86_64"): metal,
        clean_dep("@third_party//filament:filament_uses_opengl_ios_x86_64"): opengl,
        clean_dep("@third_party//filament:linux"): opengl,
        "//conditions:default": opengl,
    })

def if_metal(a, otherwise = []):
    return by_backend(metal = a, opengl = otherwise, vulkan = otherwise)

def if_vulkan(a, otherwise = []):
    return by_backend(metal = otherwise, opengl = otherwise, vulkan = a)

def if_optimized(a, otherwise = []):
    return select({
        clean_dep("@third_party//filament:opt"): a,
        "//conditions:default": otherwise,
    })

# A looser set of copts which is compatible with e.g. //util/status
def filament_google3_copts():
    out_copts = [
        "-Wno-reorder",
        "-Wno-unused-variable",
        "-fno-exceptions",
        "-Wno-unused-local-typedef",
        "-Wno-c++20-designator",
        "-Wno-implicit-fallthrough",
        "-Wno-string-conversion",
        "-Wno-pass-failed",  # clang misses #pragma loop optimizations
    ]
    return out_copts

def filament_copts():
    """Compilation flags for Filament

    Returns:
        A list of copts for Filament.
    """
    out_common_copts = filament_google3_copts() + [
        "-Wno-ctad-maybe-unsupported",
    ]
    out_copts_bitcode_optimized = out_common_copts + [
        "-fvisibility=hidden",
        "-DNDEBUG",
        # Android/iOS toolchains default to size-optimized in opt builds,
        # but Filament assumes optimizations like inlining and vectorization
        # are happening, so turn those back on.
        "-O3",
    ]
    out_copts_optimized = out_copts_bitcode_optimized + [
        "-ffunction-sections",
        "-fdata-sections",
    ]
    out_copts_unoptimized = out_common_copts + [
        # Optimize for small code size in non-opt builds.
        # In opt builds, this flag actually makes the size larger.
        # However, we still care about code-size in non-opt builds.
        "-Os",
    ]
    extra_arm_copts = [
        "-fno-rtti",
        "-ffast-math",
        "-fno-finite-math-only",
        "-ffp-contract=fast",
    ]

    return select({
        clean_dep("@third_party//filament:bitcode_and_optimized"): out_copts_bitcode_optimized,
        clean_dep("@third_party//filament:opt"): out_copts_optimized,
        "//conditions:default": out_copts_unoptimized,
    }) + select({
        "@third_party//filament:android_arm64": extra_arm_copts,
        "@third_party//filament:ios_arm64": extra_arm_copts,
        "//conditions:default": [],
    })

def filament_defines():
    out_defines = if_ios([
        "FILAMENT_IOS=1",
        "FILAMENT_TARGET_MOBILE=1",
    ]) + if_ios_simulator([
        "FILAMENT_IOS_SIMULATOR",
        "FILAMENT_TARGET_MOBILE=1",
    ]) + if_metal([
        "FILAMENT_SUPPORTS_METAL",
    ]) + if_android([
        "__ANDROID_UNAVAILABLE_SYMBOLS_ARE_WEAK__",
        "FILAMENT_TARGET_MOBILE=1",
    ]) + if_matdbg(
        ["FILAMENT_ENABLE_MATDBG=1"],
        ["FILAMENT_ENABLE_MATDBG=0"],
    ) + if_fgviewer(
        ["FILAMENT_ENABLE_FGVIEWER=1"],
        ["FILAMENT_ENABLE_FGVIEWER=0"],
    ) + if_systrace(
        ["SYSTRACE_TAG=1"],
        ["SYSTRACE_TAG=0"],
    ) + if_android_perfetto(
        [
            "FILAMENT_ENABLE_PERFETTO=1",
            "FILAMENT_TRACING_ENABLED=1",
        ],
        ["FILAMENT_TRACING_ENABLED=0"],
    ) + [
        # Disable GTAO on g3 due to size increase.
        "FILAMENT_DISABLE_GTAO=1",
        "FILAMENT_RELAXED_CORRECTNESS_ASSERTIONS=1",
        # Enable Abseil logging in g3.
        
    ]
    return out_defines

def filament_linkopts():
    out_linkopts = [
        "-Wl,--icf=all",
        "-Wl,--gc-sections",
        "-Wl,-Bsymbolic-functions",
    ]
    return out_linkopts

def filament_jni_copts():
    out_common_copts = [
        "-Wno-reorder",
        "-Wno-unused-variable",
        "-fno-exceptions",
        "-Wno-unused-local-typedef",
        "-Wno-c++20-designator",
    ]
    out_copts_optimized = out_common_copts + [
        # Generate separate linker section for each function or data object.
        # This is needed to make --gc-sections work.
        "-fdata-sections",
        "-ffunction-sections",
        # Don't export symbols from .so by default.
        # (Exported symbols must be explicitly annotated, e.g. with JNI_EXPORT.)
        "-fvisibility-inlines-hidden",
        "-fvisibility=hidden",
        # Don't generate run-time type info. Reduces size, and we don't need it.
        "-fno-rtti",
    ]
    out_copts_unoptimized = out_common_copts + [
        # Optimize for small code size in non-opt builds.
        # In opt builds, this flag actually makes the size larger.
        # However, we still care about code-size in non-opt builds.
        "-Os",
    ]

    return filament_copts() + select({
        clean_dep("@bazel_tools//tools/compilation_mode:opt"): out_copts_optimized,
        "//conditions:default": out_copts_unoptimized,
    })

def uberarchive_args(shadingmodel, blending):
    return '''-TCUSTOM_PARAMS="// no custom params" \
        -TCUSTOM_VERTEX="// no custom vertex" \
        -TCUSTOM_FRAGMENT="// no custom fragment" \
        -TDOUBLESIDED=false \
        -TTRANSPARENCY=default \
        -TSHADINGMODEL={SHADINGMODEL} \
        -TBLENDING={BLENDING}'''.format(SHADINGMODEL = shadingmodel, BLENDING = blending)

def copy_files(
        name = "",
        srcs = [],
        outs = []):
    for i in range(len(srcs)):
        native.genrule(
            name = "%s_%s_rule" % (name, outs[i]),
            srcs = [srcs[i]],
            outs = [outs[i]],
            cmd = "cp $(SRCS) $(OUTS)",
        )

def filament_jni_linkopts():
    out_linkopts = [
        # Identical Code Folding
        "-Wl,--icf=all",
        # Don't link in unused sections.
        "-Wl,--gc-sections",
        "-Wl,-Bsymbolic-functions",
    ]
    return out_linkopts

BUILTIN_MATERIAL_INCLUDES = [
    "antiAliasing/fxaa/fxaa.fs",
    "colorGrading/colorGrading.fs",
    "dof/dofUtils.fs",
    "fsr/ffx_a.h",
    "fsr/ffx_fsr1.h",
    "fsr/ffx_fsr1_mobile.fs",
    "separableGaussianBlur.fs",
    "separableGaussianBlur.vs",
    "sgsr/sgsr1_shader_mobile.fs",
    # Disable GTAO due to size increase.
    # "ssao/gtaoImpl.fs",
    "ssao/saoImpl.fs",
    "ssao/ssaoUtils.fs",
    "ssao/ssct.fs",
    "ssao/ssctImpl.fs",
    "utils/depthUtils.fs",
    "utils/geometry.fs",
]

# Each of the entries represents a material binary target. This target name is also the directory
# name under `src/materials` that contains the material source files.
#
# The `path_prefix` is used to specify a subdirectory under `src/materials` for the material
# sources. This is used to organize the material source files in a subdirectory while still
# referencing the material by its base name in the build process.

# For example, the `bloom` set of materials is defined as:
#{
#     "bloom": {
#         "path_prefix": "bloom",
#         "files": [
#             "bloomDownsample",
#             "bloomDownsample2x",
#             "bloomDownsample9",
#             "bloomUpsample",
#         ],
#     },
# }
#
# This indicates that the material source files are named `bloomDownsample.mat`, `bloomDownsample2x.mat`, etc.
# And that they are all located in a directory named `src/materials/bloom`.
BUILTIN_MATERIAL_NAMES = {
    # This is a set of basic materials that are not separated into post-processing features.
    # This group is called "materials" for backward compatibility. (But could be renamed to "base")
    "materials": {
        "path_prefix": "",
        "files": [
            "blitArray",
            "debugShadowCascades",
            "defaultMaterial",
            "blitDepth",
            "blitLow",
            "clearDepth",
            "resolveDepth",
            "shadowmap",
            "skybox",
            "separableGaussianBlur",
            "vsmMipmap",
        ],
    },
    "colorGrading": {
        "path_prefix": "colorGrading",
        "files": [
            "colorGrading",
            "colorGradingAsSubpass",
            "customResolveAsSubpass",
        ],
    },
    "dof": {
        "path_prefix": "dof",
        "files": [
            "dof",
            "dofDownsample",
            "dofCoc",
            "dofCombine",
            "dofTiles",
            "dofTilesSwizzle",
            "dofDilate",
            "dofMipmap",
            "dofMedian",
        ],
    },
    "bloom": {
        "path_prefix": "bloom",
        "files": [
            "bloomDownsample",
            "bloomDownsample2x",
            "bloomDownsample9",
            "bloomUpsample",
        ],
    },
    "fsr": {
        "path_prefix": "fsr",
        "files": [
            "fsr_easu",
            "fsr_easu_mobile",
            "fsr_easu_mobileF",
            "fsr_rcas",
        ],
    },
    "sgsr": {
        "path_prefix": "sgsr",
        "files": [
            "sgsr1",
        ],
    },
    "ssao": {
        "path_prefix": "ssao",
        "files": [
            "sao",
            "saoBentNormals",
            "bilateralBlur",
            "bilateralBlurBentNormals",
            "mipmapDepth",
            # Disable GTAO due to size increase.
            # "ssao/gtao",
            # "ssao/gtaoBentNormals",
        ],
    },
    "flare": {
        "path_prefix": "flare",
        "files": [
            "flare",
        ],
    },
    "taa": {
        "path_prefix": "antiAliasing/taa",
        "files": [
            "taa",
        ],
    },
    "fxaa": {
        "path_prefix": "antiAliasing/fxaa",
        "files": [
            "fxaa",
        ],
    },
    "fog": {
        "path_prefix": "fog",
        "files": [
            "fog",
        ],
    },
}

BUILTIN_MATERIAL_NAMES_FL0 = [
    "defaultMaterial",
    "skybox",
]

BUILTIN_MATERIAL_NAMES_MULTIVIEW = [
    "defaultMaterial",
    "skybox",
]

def filament_get_internal_material_targets():
    """Returns the list of internal material targets"""
    return BUILTIN_MATERIAL_NAMES.keys()

def _flatten(lists):
    flattened = []
    for list in lists:
        flattened.extend(list)
    return flattened

# buildifier: disable=unused-variable
def _get_material_compilation_steps(output_path, enable_fl0, enable_multiview, metal_precompile_mode):
    """Given an output path and options (FL0, MV, MetalPrecomp), enumerate builtin materials and return a dict of compilation steps for them."""

    def _options_for_precompile_mode():
        STANDARD_PRECOMPILER_ARGS = ["--min-ios-version=15.0", "--xctool-allow-stderr-output-on-success"]
        if metal_precompile_mode == None or metal_precompile_mode == "none":
            return (False, [])
        elif metal_precompile_mode == "device":
            return (True, STANDARD_PRECOMPILER_ARGS)
        elif metal_precompile_mode == "sim":
            return (True, STANDARD_PRECOMPILER_ARGS + ["--mobile-is-simulator"])
        else:
            fail("unsupported metal_precompile_mode: %s" % metal_precompile_mode)

    def _make_matc_filamat_filename(base_filamat_filename):
        """Given an expected output path such as a/b/c/d.filamat, returns a/b/c/matc_d.filamat"""
        base, sep, filename = base_filamat_filename.rpartition("/")
        return base + sep + "matc_" + filename

    def _make_single_step(material_name, prefix_path):
        """Given a material name with no extension (e.g. 'ssao/mipmapDepth'), return a dict describing how to compile it."""

        # We put the output of matc/cmat in a directory matching its path prefix, if any.
        material_name_impl = material_name if len(prefix_path) == 0 else prefix_path + "/" + material_name
        src_path = "src/materials/%s.mat" % material_name_impl
        filamat_out_path = "%s/%s.filamat" % (output_path, material_name_impl)
        deploy_path = "$$DEPLOY/%s.filamat" % material_name_impl
        enable_metal_precompile, precompiler_args = _options_for_precompile_mode()

        compilation_step = {}
        compilation_step["src"] = src_path
        compilation_step["outs"] = [filamat_out_path]
        compilation_step["deploys"] = [deploy_path]

        if enable_metal_precompile:
            matc_filamat_out_filename = _make_matc_filamat_filename(filamat_out_path)
            matc_deploy_path = _make_matc_filamat_filename(deploy_path)
            compilation_step["outs"].append(matc_filamat_out_filename)
            compilation_step["cmds"] = [
                "$$MATC $$ESSL1 -a $$API -p $$PLATFORM -o %s $(location %s)" % (matc_deploy_path, src_path),
                "$$MATEDIT -i %s -o %s -t metal -p external-compile -- $$PRECOMPILE %s" % (matc_deploy_path, deploy_path, " ".join(precompiler_args)),
            ]
        else:
            compilation_step["cmds"] = ["$$MATC $$ESSL1 -a $$API -p $$PLATFORM -o %s $(location %s)" % (deploy_path, src_path)]

        if material_name in BUILTIN_MATERIAL_NAMES_FL0 and enable_fl0:
            if enable_metal_precompile:
                fail("can't use enable_fl0 and enable_metal_precompile simultaneously")
            compilation_step["outs"].append("%s/%s_fl0.filamat" % (output_path, material_name_impl))
            compilation_step["deploys"].append("$$DEPLOY/%s_fl0.filamat" % material_name_impl)
            compilation_step["cmds"].append("$$MATC $$ESSL1 -a $$API -p $$PLATFORM -PfeatureLevel=0 -o $$DEPLOY/%s_fl0.filamat $(location src/materials/%s.mat)" % (material_name, material_name))

        if material_name in BUILTIN_MATERIAL_NAMES_MULTIVIEW and enable_multiview:
            if enable_metal_precompile:
                fail("can't use enable_multiview and enable_metal_precompile simultaneously")
            compilation_step["outs"].append("%s/%s_multiview.filamat" % (output_path, material_name_impl))
            compilation_step["deploys"].append("$$DEPLOY/%s_multiview.filamat" % material_name_impl)
            compilation_step["cmds"].append("$$MATC $$ESSL1 -a $$API -p $$PLATFORM -PstereoscopicType=multiview -o $$DEPLOY/%s_multiview.filamat $(location src/materials/%s.mat)" % (material_name, material_name))
        return compilation_step

    return {
        k: [_make_single_step(material_name, BUILTIN_MATERIAL_NAMES[k]["path_prefix"]) for material_name in BUILTIN_MATERIAL_NAMES[k]["files"]]
        for k in BUILTIN_MATERIAL_NAMES
    }

# This should really be in filament/filament/BUILD but for some reason Blaze
# doesn't allow macro definitions in build files nor does it allow variables to
# be defined within the scope of a list comprehension without defining a macro.
#
# TODO: (broken link) - This function could use some simplification, since FL0 or Multiview is only
# ever enabled for OpenGL, and MetalPrecompileMode is Metal only. Consider replacing all args
# after `material_api` with an `options` dict (or **kwargs) that we look up options in as needed.
#
# TODO: (broken link) - Generating directory trees in a genrule is an antipattern, but it's currently
# required because resgen encodes relative pathnames in its output. If we added an option to resgen
# to explicitly specify the resource name for input files, then this "do everything" genrule could
# be replaced with N calls to filament_matc() and a single filament_resgen() rule, which would both
# eliminate the need for artifact subdirectories and enable parallel compilation at build time.
#
# buildifier: disable=unnamed-macro
def filament_generate_materials(
        platform,
        api,
        material_api,
        enable_fl0,
        enable_multiview,
        metal_precompile_mode):
    """Generates a genrule target that creates source code for an embedded material library.

    Args:
      platform: The platform to generate materials for. One of "mobile" or "desktop".
      api: A user-friendly API package name, used in the output path. (This doesn't directly affect
        what shaders are generated; it's just used to disambiguate targets while we're using a
        genrule to generate this.)
      material_api: The backend API to generate materials for.
        One of "opengl", "metal", "vulkan", or "all".
      enable_fl0: Whether to generate Feature Level 0 compatible materials.
      enable_multiview: Whether to generate stereoscopic-compatible materials.
      metal_precompile_mode: Whether to generate precompiled Metal shaders. One of "none", "device",
        or "sim". (None is also interpreted as "none".)
    """

    def _subdir_for_precompile_mode():
        if metal_precompile_mode == None or metal_precompile_mode == "none":
            return (False, "noMPC")
        elif metal_precompile_mode == "device":
            return (True, "yesMPC")
        elif metal_precompile_mode == "sim":
            return (True, "yesMPCSim")
        else:
            fail("unsupported metal_precompile_mode: %s" % metal_precompile_mode)

    RESGEN_OUTPUT_EXTENSIONS = ["bin", "h", "c", "S", "apple.S"]

    fl0 = "yesFL0" if enable_fl0 else "noFL0"
    multiview = "yesMV" if enable_multiview else "noMV"
    needs_precompile_tools, precompile = _subdir_for_precompile_mode()
    out_path = "%s/%s/%s/%s/%s/generated/resources" % (platform, api, fl0, multiview, precompile)

    compilation_steps_per_target = _get_material_compilation_steps(
        output_path = out_path,
        enable_fl0 = enable_fl0,
        enable_multiview = enable_multiview,
        metal_precompile_mode = metal_precompile_mode,
    )
    for target, compilation_steps in compilation_steps_per_target.items():
        material_srcs = [step["src"] for step in compilation_steps]
        material_outs = _flatten([step["outs"] for step in compilation_steps])
        material_cmds = _flatten([step["cmds"] for step in compilation_steps])
        material_deploys = _flatten([step["deploys"] for step in compilation_steps])

        all_srcs = material_srcs + ["src/materials/%s" % name for name in BUILTIN_MATERIAL_INCLUDES]
        all_outs = material_outs + ["%s/%s.%s" % (out_path, target, ext) for ext in RESGEN_OUTPUT_EXTENSIONS]

        cmds = [
            "MATC=$(location @third_party//filament:matc)",
            "RESGEN=$(location @third_party//filament:resgen)",
        ]
        if needs_precompile_tools:
            cmds += [
                "MATEDIT=$(location @third_party//filament:matedit)",
                "PRECOMPILE=$(location @third_party//filament/build_tools:compile_metal_shader)",
            ]
        cmds += [
            "PLATFORM=%s" % platform,
            "API=%s" % material_api,
            "ESSL1=%s" % ("" if enable_fl0 else "-1"),
            "DEPLOY=`dirname $(location %s/%s.bin)`" % (out_path, target),
            "OUTPUT_NAME=%s" % target,
        ]
        cmds += material_cmds
        cmds.append("$$RESGEN -cp $$OUTPUT_NAME --deploy=$$DEPLOY " + " ".join(material_deploys))

        tools = [
            "@third_party//filament:matc",
            "@third_party//filament:resgen",
            "@third_party//filament:shader_srcs",
        ]
        exec_compatible_with = []

        if needs_precompile_tools:
            tools += [
                "@third_party//filament/build_tools:compile_metal_shader",
                "@third_party//filament:matedit",
            ]
            exec_compatible_with.append("@platforms//os:macos")

        native.genrule(
            name = "_generate_%s_%s_%s_%s_%s_%s_materials" % (platform, api, fl0, multiview, precompile, target),
            srcs = all_srcs,
            outs = all_outs,
            cmd = " && ".join(cmds),
            tools = tools,
            exec_compatible_with = exec_compatible_with,
            visibility = ["//visibility:private"],
        )
