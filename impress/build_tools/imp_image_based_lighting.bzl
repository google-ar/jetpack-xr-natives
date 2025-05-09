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

"""Generates ibl (image-based lighting) assets from an environment map image"""

load("@com_google_impress//build_tools:generate_ibl.bzl", "generate_ibl_rule")

def imp_image_based_lighting(
        name,
        src,
        size = 256,
        ibl_samples = 1024,
        blur_roughness = 0.05,
        include_skybox_cubemap = False,
        exclude_lighting_cubemap = False,
        exclude_spherical_harmonics = False):
    """
    Takes a source environment map image (supports .exr, .psd, .hdr, .png) and generate spherical harmonics and cubemap mipmap levels.

    This is intended for use with imp_assets, ie:

    imp_image_based_lighting(
      name = "my_ibl",
      src = "my_env_map.exr",
      size = 256,
      ibl_samples = 1024,
      blur_roughness = 0.05,
    )

    imp_assets(
      name = "my_ibl_assets",
      srcs = [
        ":my_ibl",
      ],
    )

    imp assets can then be added to a cc_library rule.

    Args
      name: The name of the ibl asset.  The output file will have the name of %name%.zip
      source_image: Text input for the material.  Internally, this runs //third_party/filament:cmgen on the source image and then uses //third_party/zip:zip to zip the output file.
      size: Size of the output cubemaps (base level), 256 by default. Must be a power-of-two.
      ibl_samples:  Number of samples to use for IBL integrations, 1024 by default.
      blur_roughness: Blurs the skybox cubemap before saving the faces using the roughness blur. blur_roughness is the blur strength, 0.05 by default. Must be between 0.0 and 1.0.

    By default, include_skybox_cubemap, exclude_lighting_cubemap and exclude_spherical_harmonics are False.
    """

    generate_ibl_rule(
        name = name,
        source_image = src,
        size = size,
        ibl_samples = ibl_samples,
        blur_roughness = str(blur_roughness),
        output_path = "%s.zip" % name,
        include_skybox_cubemap = include_skybox_cubemap,
        exclude_lighting_cubemap = exclude_lighting_cubemap,
        exclude_spherical_harmonics = exclude_spherical_harmonics,
    )
