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

"""Generates Material Assets for applications using the Impress Framework"""

load("@com_google_impress//core/loader/data:generic_material.bzl", "process_and_build_material")

DEFAULT_VARIANT_FILTER = "vsm,fog,ssr,stereo"
DEFAULT_XR_VARIANT_FILTER = "vsm,fog,ssr"

def imp_material(
        name,
        src,
        replacements = None,
        includes = [],
        visibility = None,
        variant_filter = select({
            "@com_google_impress//core:imp_include_stereo_variant_by_default": DEFAULT_XR_VARIANT_FILTER,
            "//conditions:default": DEFAULT_XR_VARIANT_FILTER,
        }),
        defines = [],
        **kwargs):
    """Creates a binary material asset

    This is intended for use with imp_assets, ie:
    imp_material(
      name = "my_material",
      src = "my_material.mat",
      includes = [":samplers.glsl"],
    )

    imp_assets(
      name = "my_application_assets",
      srcs = [":my_material",],
    )
    imp assets can then be added to a cc_library rule.

    Args:
      name: The name of the material asset.  The output file will have the extension .cmat
      src: Text input for the material.  The text will be processed to do replacements and add included files, then compiled into a binary asset.
      replacements: Performs variable replacement during processing.
      includes: Text files to be concatenated during processing.  Includes within includes are not supported.
      visibility: The visibility attribute on a rule controls whether the rule can be used by other packages.
      variant_filter: The list of variants to filter out of compiled materials (for size savings)
      defines: Defines passed through to the material compiler that can be used from a .mat file.
      **kwargs: Other parameters to pass to the material compiler. ("optimization" and
        "enable_metal_postprocessing" are typically passed this way.)
    """

    process_and_build_material(
        name = name,
        template_file = src,
        replacements = replacements,
        incs = includes,
        visibility = visibility,
        variant_filter = variant_filter,
        defines = defines,
        **kwargs
    )
