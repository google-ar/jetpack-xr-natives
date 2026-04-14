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

"""Rule for creating a generic material from a template"""

load("@bazel_skylib//rules:copy_file.bzl", "copy_file")
load("@third_party//filament:filament.bzl", "by_backend")
# Placeholder: load postprocess_metal_shaders
load("@com_google_impress//build_tools:imp.bzl", "if_enable_stereo_type_multiview", "if_imp_split_engine_allow_experimental_apis")

# Minimum iOS version to target when generating Metal AIR bitcode.
_METAL_POSTPROCESSING_MINIMUM_IOS_VERSION = "15.0"

def _process_material_impl(ctx):
    # Creates a copy of the read-only replacement dict.
    replacements = dict(ctx.attr.replacements)

    # Adds a replacement for samplerExternal, which is only fully supported on android.
    replacements["{SAMPLER_EXTERNAL}"] = (
        "samplerExternal" if ctx.attr.platform == "android" else "sampler2d"
    )

    replacements["{OCCLUSION_HELPERS_GLSL}"] = (
        "occlusion_helpers_ar_kit.glsl" if ctx.attr.platform == "ios" else "occlusion_helpers_ar_core.glsl"
    )

    ctx.actions.expand_template(
        template = ctx.file.template_file,
        output = ctx.outputs.processed_material,
        substitutions = replacements,
    )

    outputs = []
    for inc_file in ctx.files.incs:
        processed_inc_file = ctx.actions.declare_file(
            inc_file.basename,
            sibling = ctx.outputs.processed_material,
        )
        outputs.append(processed_inc_file)

        ctx.actions.expand_template(
            template = inc_file,
            output = processed_inc_file,
            substitutions = replacements,
        )

    return [
        DefaultInfo(
            files = depset([ctx.outputs.processed_material]),
            runfiles = ctx.runfiles(
                files = outputs,
                collect_data = True,
                collect_default = True,
            ),
        ),
    ]

_process_material_rule = rule(
    implementation = _process_material_impl,
    attrs = {
        "platform": attr.string(),
        "template_file": attr.label(allow_single_file = True),
        "replacements": attr.string_dict(default = {}),
        "incs": attr.label_list(allow_files = True, default = []),
        "processed_material": attr.output(),
    },
)

# Preprocesses a material file by replacing strings using regex.
# The processed file will have the extension ".mat".
def process_material(name, template_file, incs = [], output = None, replacements = None):
    _process_material_rule(
        name = name,
        template_file = template_file,
        incs = incs,
        replacements = replacements,
        processed_material = output if output else name + "_output/" + name + ".mat",
        platform = select({
            "@third_party//filament:wasm": "wasm",
            "@third_party//filament:ios": "ios",
            "@third_party//filament:android": "android",
            "//conditions:default": None,
        }),
    )

DEFAULT_OPTIMIZATION_FLAG = "-O"

def _optimization_flag_for_optimization(optimization):
    if optimization == "performance":
        return "-O"
    elif optimization == "size":
        return "--optimize-size"
    elif optimization == "unoptimized":
        return "--optimize-none"
    elif optimization == None or optimization == "":
        return DEFAULT_OPTIMIZATION_FLAG
    fail("optimization string provided (%s) must be one of: 'performance', 'size', 'unoptimized', None" % optimization)

def _build_material_impl(ctx):
    api = ctx.attr.api
    platform = ctx.attr.platform
    optimization_flag = _optimization_flag_for_optimization(ctx.attr.optimization)
    variant_filter = "--variant-filter %s" % ctx.attr.variant_filter if ctx.attr.variant_filter else ""

    filamat_command = " ".join([
        ctx.executable.matc.path,
        optimization_flag,
        variant_filter,
        "" if ctx.attr.include_essl1 else "-1",
        "-PstereoscopicType=multiview" if ctx.attr.enable_multiview else "",
        "-a %s" % api,
        "-p %s" % platform,
        "--include-source-mat" if ctx.attr.include_source_mat else "",
        "-o %s" % ctx.outputs.compiled_material.path,
        ctx.file.material_source.path,
    ])

    for define in ctx.attr.defines:
        filamat_command += " -D" + define

    ctx.actions.run_shell(
        tools = [ctx.executable.matc],
        inputs = depset([ctx.file.material_source] + ctx.files.incs, transitive = [ctx.attr.material_source[DefaultInfo].default_runfiles.files]),
        outputs = [ctx.outputs.compiled_material],
        mnemonic = "ImpressBuildMaterial",
        command = filamat_command,
    )

    return [
        DefaultInfo(
            files = depset([ctx.outputs.compiled_material]),
        ),
    ]

_build_material_rule = rule(
    implementation = _build_material_impl,
    attrs = {
        "incs": attr.label_list(allow_files = True, default = []),
        "matc": attr.label(default = "@third_party//filament:matc", executable = True, cfg = "exec"),
        "material_source": attr.label(allow_single_file = True),
        "compiled_material": attr.output(),
        "api": attr.string(),
        "platform": attr.string(),
        "variant_filter": attr.string(),
        "defines": attr.string_list(),
        "optimization": attr.string(),
        "include_essl1": attr.bool(default = False),
        "enable_multiview": attr.bool(default = False),
        "include_source_mat": attr.bool(default = False),
    },
)

def _postprocess_metal_shaders_noop_impl(ctx):
    ctx.actions.symlink(
        output = ctx.outputs.output_material,
        target_file = ctx.file.input_material,
    )
    return [DefaultInfo(files = depset([ctx.outputs.output_material]))]

# Stub version of _postprocess_metal_shaders_noop() that just symlinks from input to output.
# This replaces the real _postprocess_metal_shaders_noop() when Impress is built for Moohan on GoB.
# buildifier: disable=unused-variable
_postprocess_metal_shaders_noop = rule(
    implementation = _postprocess_metal_shaders_noop_impl,
    attrs = {
        "input_material": attr.label(allow_single_file = True),
        "output_material": attr.output(),
        "minimum_ios_version": attr.string(),
        "preserve_text_shaders": attr.bool(),
        "backend_is_metal": attr.bool(),
        "fast_math": attr.bool(),
    },
)

def _enable_metal_postprocessing_by_default():
    # TODO: (broken link) - Enable metal postprocessing by default when ready.
    # We may end up shipping a partial release of it where we branch based on
    # native.package_name() to avoid postprocessing for MapCore shaders.
    return False

def _emit_single_matc_target(
        name,
        material_source,
        compiled_material,
        api,
        force_feature_level_zero = False,
        include_source_mat = False,
        **kwargs):
    _FORBIDDEN_ARGS = ["platform", "include_essl1", "enable_multiview"]
    for arg in _FORBIDDEN_ARGS:
        if arg in kwargs:
            fail("_emit_single_matc_target overrides %s, do not pass it" % arg)

    _build_material_rule(
        name = name,
        material_source = material_source,
        compiled_material = compiled_material,
        api = api,
        platform = select({
            "@third_party//filament:wasm": "mobile",
            "@third_party//filament:ios": "mobile",
            "@third_party//filament:android": "mobile",
            "//conditions:default": "all",
        }),
        include_essl1 = True if force_feature_level_zero else select({
            "@third_party//filament:filament_uses_gles2_android": True,
            "@third_party//filament:filament_uses_opengl_egl_headless_fl0": True,
            "@third_party//filament:filament_uses_gles3_android": False,
            "@third_party//filament:filament_uses_vulkan_android": False,
            "@third_party//filament:wasm_feature_level_0": True,
            "@third_party//filament:android": True,
            "//conditions:default": False,
        }),
        enable_multiview = if_enable_stereo_type_multiview(True, False),
        include_source_mat = include_source_mat,
        **kwargs
    )

def process_and_build_material(
        name,
        template_file,
        replacements = None,
        incs = [],
        variant_filter = None,
        defines = [],
        visibility = None,
        optimization = None,
        enable_metal_postprocessing = None,
        metal_postprocessing_fast_math = None,
        preserve_text_shaders = None,
        force_feature_level_zero = False,
        include_source_mat = None):
    """Preprocesses a material with replacements and includes, and builds it using filamat.

    Includes within includes are not supported.

    Args:
        name: The name to be assigned to the outermost rule, that produces the final compiled
           material. The output file from this rule will have the extension ".cmat".
        template_file: The material source code to process
        replacements: Optional. A dictionary of strings for preprocessing, mapping placeholders
           to expansions.
        incs: Optional. A list of includes expected to be used by the material; each include
           will undergo preprocessing as well before being concatenated and inlined.
        variant_filter: Optional. A string containing a comma-separated list of variants to
           filter out of the compiled material, for size savings.
        defines: Optional. Defines to be passed along to matc.
        visibility: The visibility attribute on a rule controls whether the rule can be used
           by other packages.
        optimization: Optional. One of "performance", "size", "unoptimized", or None, to control
           optimization in matc. (Leaving it unset, or None, uses DEFAULT_OPTIMIZATION_FLAG.)
        enable_metal_postprocessing: Optional. Whether to pre-compile Metal shaders to LLVM bitcode.
           The default (None) allows this rule to decide whether or not to enable precompiling.
           Pass True or Force to force precompilation / force text shaders only.
        metal_postprocessing_fast_math: Optional. Whether to enable fast math when pre-compiling
           Metal shaders. The default (None) allows this rule to decide whether or not to enable
           fast math. Pass True or Force to force fast math.
        preserve_text_shaders: Optional. Whether to preserve MSL text shaders when Metal post-
           processing is enabled. By default, text shaders are removed; pass True to preserve them.
           This flag is ignored if Metal post-processing is disabled.
        force_feature_level_zero: Optional. Whether to force only ESSL 1.0 materials to be compiled.
        include_source_mat: Optional. Whether to include the source code in the material. The
           default (None) allows this rule to include the source in experimental builds.
    """
    is_opengl = by_backend(
        metal = False,
        opengl = True,
        vulkan = False,
        gl_vulkan = True,
    )

    if not is_opengl and force_feature_level_zero:
        fail("force_feature_level_zero only works with OpenGL")

    if enable_metal_postprocessing == None:
        enable_metal_postprocessing = _enable_metal_postprocessing_by_default()

    # Custom materials are an experimental feature that requires the source code in the cmat.
    if include_source_mat == None:
        include_source_mat = if_imp_split_engine_allow_experimental_apis(True, otherwise = False)

    process_target = "process_" + name
    process_material(
        name = process_target,
        template_file = template_file,
        incs = incs,
        replacements = replacements,
    )

    _emit_single_matc_target(
        name = "opengl_cmat_%s" % name,
        material_source = ":%s" % process_target,
        compiled_material = "opengl/%s.cmat" % name,
        api = "opengl",
        variant_filter = variant_filter,
        defines = defines,
        optimization = optimization,
        force_feature_level_zero = force_feature_level_zero,
        include_source_mat = include_source_mat,
    )

    if not force_feature_level_zero:
        _emit_single_matc_target(
            name = "vulkan_cmat_%s" % name,
            material_source = ":%s" % process_target,
            compiled_material = "vulkan/%s.cmat" % name,
            api = "vulkan",
            variant_filter = variant_filter,
            defines = defines,
            optimization = optimization,
            include_source_mat = include_source_mat,
        )

        _emit_single_matc_target(
            name = "gl_vulkan_cmat_%s" % name,
            material_source = ":%s" % process_target,
            compiled_material = "gl_vulkan/%s.cmat" % name,
            api = "all",
            variant_filter = variant_filter,
            defines = defines,
            optimization = optimization,
            include_source_mat = include_source_mat,
        )

    if not force_feature_level_zero:
        if enable_metal_postprocessing:
            metal_matc_target_name = "metal_matc_%s" % name
            _emit_single_matc_target(
                name = metal_matc_target_name,
                material_source = ":%s" % process_target,
                compiled_material = "metal_matc/%s.cmat" % name,
                api = "metal",
                variant_filter = variant_filter,
                defines = defines,
                optimization = optimization,
                force_feature_level_zero = force_feature_level_zero,
                include_source_mat = include_source_mat,
            )
            _postprocess_metal_shaders_noop(
                name = "metal_cmat_%s" % name,
                input_material = ":%s" % metal_matc_target_name,
                output_material = "metal/%s.cmat" % name,
                minimum_ios_version = _METAL_POSTPROCESSING_MINIMUM_IOS_VERSION,
                preserve_text_shaders = preserve_text_shaders,
                backend_is_metal = True,
                fast_math = metal_postprocessing_fast_math,
            )
        else:
            _emit_single_matc_target(
                name = "metal_cmat_%s" % name,
                material_source = ":%s" % process_target,
                compiled_material = "metal/%s.cmat" % name,
                api = "metal",
                variant_filter = variant_filter,
                defines = defines,
                optimization = optimization,
                force_feature_level_zero = force_feature_level_zero,
                include_source_mat = include_source_mat,
            )

    # The generic alias should automatically select a backend-specific build
    # rule based on config_settings() and copy it to a generic output name.
    if force_feature_level_zero:
        copy_file(
            name = name,
            src = ":opengl_cmat_%s" % name,
            out = "%s.cmat" % name,
            allow_symlink = True,
            visibility = visibility,
        )
    else:
        copy_file(
            name = name,
            src = by_backend(
                metal = ":metal_cmat_%s" % name,
                opengl = ":opengl_cmat_%s" % name,
                vulkan = ":vulkan_cmat_%s" % name,
                gl_vulkan = ":gl_vulkan_cmat_%s" % name,
            ),
            out = "%s.cmat" % name,
            allow_symlink = True,
            visibility = visibility,
        )
