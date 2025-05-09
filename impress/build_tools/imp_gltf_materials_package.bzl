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

"""Rule for creating prebuilt or customized material packages from a template"""

load("@rules_pkg//:pkg.bzl", "pkg_zip")
load("@com_google_impress//core/loader/data:generic_material.bzl", "process_and_build_material")
load("@com_google_sandboxed_api//sandboxed_api/bazel:embed_data.bzl", "sapi_cc_embed_data")

AVAILABLE_SHADING_MODELS = ["lit", "unlit"]
AVAILABLE_BLEND_MODES = ["masked", "opaque", "transparent", "refractive"]
AVAILABLE_SIDED = ["single_sided", "double_sided"]

# See (broken link)
FILAMENT_VARIANT_FEATURES = ["fog", "ssr", "vsm", "stereo"]
AVAILABLE_FEATURES = [
    "clearcoat",
    "sheen",
    "transmission",
    "ar_occlusion",
    "depth_clear",
] + FILAMENT_VARIANT_FEATURES

# Error messages when arguments are invalid
SHADING_MODELS_ERROR_MESSAGE = "Expected one or both of %s" % ", ".join(AVAILABLE_SHADING_MODELS)
BLEND_MODES_ERROR_MESSAGE = "Expected any combination of %s" % ", ".join(AVAILABLE_BLEND_MODES)
SIDED_ERROR_MESSAGE = "Expected one or both of %s" % ", ".join(AVAILABLE_SIDED)
FEATURES_ERROR_MESSAGE = "Expected any combination of %s" % ", ".join(AVAILABLE_FEATURES)

WHITE_FALLBACK_SAMPLER_INDEX = 16
NORMAL_FALLBACK_SAMPLER_INDEX = 17

def _validate_materials_arguments(
        user_args,
        available_options,
        error_message,
        is_empty_user_args_valid = False):
    if not is_empty_user_args_valid and not user_args:
        fail(error_message)
    for arg in user_args:
        if arg not in available_options:
            fail(error_message)

def imp_gltf_materials_package(
        name,
        shading_models,
        blend_modes,
        sided,
        features,
        **kwargs):
    """ Generates a customized materials package that includes or excludes certain features.

    Args:
        name: Name of materials package
        shading_models: one or both of ["lit", "unlit"]
        blend_modes: any combination of ["masked", "opaque", "transparent", "refractive"]
        sided: one or both of ["single_sided", "double_sided"]
        features: any combination of
            ["clearcoat", "sheen", "transmission", "ar_occlusion", "fog", "ssr", "vsm", "depth_clear"]
        **kwargs: Other arguments to pass to the material compiler rule.
            ("optimization" and "enable_metal_postprocessing" are typically passed this way.)
    """
    _validate_materials_arguments(shading_models, AVAILABLE_SHADING_MODELS, SHADING_MODELS_ERROR_MESSAGE)
    _validate_materials_arguments(blend_modes, AVAILABLE_BLEND_MODES, BLEND_MODES_ERROR_MESSAGE)
    _validate_materials_arguments(sided, AVAILABLE_SIDED, SIDED_ERROR_MESSAGE)
    _validate_materials_arguments(features, AVAILABLE_FEATURES, FEATURES_ERROR_MESSAGE, is_empty_user_args_valid = True)

    BLEND_MODE_REPLACEMENTS = {
        "masked": "blending: masked,",
        "opaque": "blending: opaque,",
        "transparent": "blending: fade,",
        "refractive": "blending: opaque," +
                      "refractionType: solid," +
                      "refractionMode: screenspace,",
    }

    AR_OCCLUSION_SAMPLER_REPLACEMENT = """\
        // Samplers reserved specifically to implement ARCore and ARKit Occlusions.
        { type : sampler2d, name : estimatedDepthTexture, precision: high },
        { type : {SAMPLER_EXTERNAL}, name : cameraTexture },
    """

    # Calculate the number of available flexible samplers to generate for use
    MAX_ES3_SAMPLERS = 16
    samplers_used_by_filament = 9
    if "fog" not in features:
        samplers_used_by_filament -= 1
    if "ssr" not in features:
        samplers_used_by_filament -= 1

    # SAMPLER_EXTERNAL counts as 2 samplers
    SAMPLERS_USED_BY_ARCORE = 3 if "ar_occlusion" in features else 0
    available_samplers = MAX_ES3_SAMPLERS - samplers_used_by_filament - SAMPLERS_USED_BY_ARCORE

    # Constants for sampler names. e.g. samplerZero, samplerOne,...
    SAMPLER_NUMBERS = ["Zero", "One", "Two", "Three", "Four", "Five", "Six", "Seven", "Eight"]

    reassignable_sampler_declaration_replacement = ""
    reassignable_sampler_switch_replacement = ""
    for i in range(available_samplers):
        i_word = SAMPLER_NUMBERS[i]
        reassignable_sampler_declaration_replacement += \
            "{ type : sampler2d, name : sampler%s },\n" % i_word
        reassignable_sampler_switch_replacement += \
            "case %s: return texture(materialParams_sampler%s, uv);\n" % (i, i_word)

    reassignable_sampler_switch_replacement += "case %s: return whiteFallbackSample();\n" % WHITE_FALLBACK_SAMPLER_INDEX
    reassignable_sampler_switch_replacement += "case %s: return normalFallbackSample();" % NORMAL_FALLBACK_SAMPLER_INDEX

    # These are the features we want to exclude from filament
    variants = [variant for variant in FILAMENT_VARIANT_FEATURES if variant not in features]
    template_defines = [feature.upper() for feature in features if feature not in FILAMENT_VARIANT_FEATURES]
    for double_sided_mode in sided:
        for shading_model in shading_models:
            for blending in blend_modes:
                this_name = "%s/%s_%s_%s" % (
                    name,
                    shading_model,
                    blending,
                    double_sided_mode,
                )
                process_and_build_material(
                    name = this_name,
                    incs = select({
                        "@com_google_impress//core:ios": [
                            Label("@com_google_impress//core/loader/data:occlusion_helpers_common.glsl"),
                            Label("@com_google_impress//core/loader/data:occlusion_helpers_ar_kit.glsl"),
                        ],
                        "//conditions:default": [
                            Label("@com_google_impress//core/loader/data:occlusion_helpers_common.glsl"),
                            Label("@com_google_impress//core/loader/data:occlusion_helpers_ar_core.glsl"),
                        ],
                    }) + [
                        Label("@com_google_impress//core/loader/data:generic_material_helpers.glsl"),
                        Label("@com_google_impress//core/materials:color_conversion_helpers.glsl"),
                    ],
                    replacements = {
                        "{NAME}": this_name,
                        "{REASSIGNABLE_SAMPLER_DECLARATION}": reassignable_sampler_declaration_replacement,
                        "{REASSIGNABLE_SAMPLER_SWITCH}": reassignable_sampler_switch_replacement,
                        "{BLENDING}": BLEND_MODE_REPLACEMENTS[blending],
                        "{DOUBLE_SIDED}": "doubleSided:true," if double_sided_mode == "double_sided" else "",
                        "{AR_OCCLUSION_SAMPLERS}": AR_OCCLUSION_SAMPLER_REPLACEMENT if "ar_occlusion" in features else "",
                        "{REASSIGNABLE_SAMPLER_COUNT}": str(available_samplers),
                    },
                    template_file =
                        Label("@com_google_impress//core/loader/data:generic_material_%s.mat.template.glsl" % shading_model),
                    variant_filter = ",".join(variants) if variants else None,
                    defines = template_defines,
                    **kwargs
                )

    mats_to_zip = [
        "%s_%s_%s.cmat" % (shading_model, blend_mode, double_sided_mode)
        for double_sided_mode in sided
        for blend_mode in blend_modes
        for shading_model in shading_models
    ]

    if "depth_clear" in features:
        process_and_build_material(
            name = "%s/%s" % (name, "depth_clear"),
            template_file =
                Label("@com_google_impress//core/loader/data:depth_clear.mat"),
            **kwargs
        )

        mats_to_zip.append("depth_clear.cmat")

    native.filegroup(
        name = "compiled_%s" % name,
        srcs = [":%s/%s" % (name, mat) for mat in mats_to_zip],
    )

    pkg_zip(
        name = "compiled_%s_zip" % name,
        srcs = ["%s/%s" % (name, mat) for mat in mats_to_zip],
        out = "compiled_%s.zip" % name,
        visibility = [
            "@com_google_impress//:__subpackages__",
            "//third_party/arcore/java/com/google/ar/core/viewer:__subpackages__",
            # TODO Remove ar/core/viewer pkg when migration is complete
            "//googlemac/iPhone/SceneViewer:__subpackages__",
        ],
    )

    sapi_cc_embed_data(
        name = "embedded_%s" % name,
        srcs = [":compiled_%s.zip" % name],
        flatten = True,
    )

    # Enable compiled materials to build with --config=arcore_tap_build_tests and make the
    # android transition.
    native.filegroup(
        name = "compiled_%s_zip_android" % name,
        srcs = [":compiled_%s.zip" % name],
        visibility = ["//visibility:public"],
    )

def generate_imp_default_gltf_materials_package(name = None):
    imp_gltf_materials_package(
        name = "imp_default_gltf_materials",
        shading_models = ["lit", "unlit"],
        blend_modes = ["masked", "opaque", "transparent", "refractive"],
        sided = ["single_sided", "double_sided"],
        features = ["clearcoat", "sheen", "transmission", "ar_occlusion", "depth_clear"],
        optimization = "size",
    )

def generate_imp_default_lite_gltf_materials_package(name = None):
    # The lite materials (used by lens) don't need refraction support, so don't include them in the zip.
    imp_gltf_materials_package(
        "imp_default_lite_gltf_materials",
        shading_models = ["lit", "unlit"],
        blend_modes = ["masked", "opaque", "transparent"],
        sided = ["single_sided", "double_sided"],
        features = [],
        optimization = "size",
    )

def generate_imp_default_performance_gltf_materials_package(name = None):
    imp_gltf_materials_package(
        name = "imp_default_performance_gltf_materials",
        shading_models = ["lit", "unlit"],
        blend_modes = ["masked", "opaque", "transparent", "refractive"],
        sided = ["single_sided", "double_sided"],
        features = ["clearcoat", "sheen", "transmission", "ar_occlusion", "depth_clear"],
        optimization = "performance",
    )
