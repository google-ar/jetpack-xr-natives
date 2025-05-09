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

"""Generates a texture from an sRGB PNG/BMP/TGA/JPEG image or list of images."""

def imp_basis_texture(name, srcs = []):
    """Creates a supercompressed texture

    This is intended for use with imp_assets, ie:
    imp_basis_texture(
      name = "my_texture",
      srcs = "my_image.png",
    )

    imp_assets(
      name = "my_application_assets",
      srcs = [":my_texture",],
    )
    imp assets can then be added to a cc_library rule.

    In a .cc use the texture:
    #include "<path>/my_application_assets.h
    ...
    GetAssetManager().LoadImage(texture_data::kMyTextureBasis);

    Args
      name: The name of the generated texture asset.  The output will have the extension basis.
      srcs: An image, or  list of images to be processed.  sRGB PNG/BMP/TGA and JPEG formats are allowed.
    """
    native.genrule(
        name = name,
        srcs = srcs,
        outs = [name + ".basis"],
        cmd = "$(location @basis_universal//:basisu)" +
              " -file $(SRCS)" +
              " -output_file \"$@\"" +
              " -mipmap",
        exec_tools = ["@basis_universal//:basisu"],
    )
