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

"""Helper functions for including various extensions into Impress."""

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@com_google_impress//build_tools:imp.bzl", "imp_copts", "imp_linkopts")

def imp_extensions(
        name,
        extensions = [],
        deps = [],
        **kwargs):
    """Generates a library with the specified loader extensions to be included in Impress.

    This is useful for creating a custom combination of loader extensions to be used in
    the impress applcation if the default ones are not appropriate.

    Current supported extensions are named as such:
    "draco", "basis", "meshopt", "verification", "behavior", "webp", "mesh_features"

    Args:
      name: The name of the cc_library generated from this rule
      extensions: The list of extensions to be included, following the names provided above.
      deps: Additional dependencies to be included if needed.
      **kwargs: Additional args passed through to the underlying rule (i.e. visibility).
    """

    if "draco" in extensions:
        deps = deps + [
            "@com_google_impress//core/loader/provider/extensions:extension_draco_impl",
        ]
    elif "draco_gltf" in extensions:
        deps = deps + [
            "@com_google_impress//core/loader/provider/extensions:extension_draco_gltf",
        ]
    else:
        deps = deps + [
            "@com_google_impress//core/loader/provider/extensions:extension_draco_noop",
        ]

    if "mesh_features" in extensions:
        deps = deps + [
            "@com_google_impress//core/loader/provider/extensions:extension_mesh_features_impl",
        ]
    else:
        deps = deps + [
            "@com_google_impress//core/loader/provider/extensions:extension_mesh_features_noop",
        ]

    if "basis" in extensions:
        deps = deps + [
            "@com_google_impress//core/loader/provider/extensions:extension_basis_impl",
        ]
    else:
        deps = deps + [
            "@com_google_impress//core/loader/provider/extensions:extension_basis_noop",
        ]

    if "meshopt" in extensions:
        deps = deps + [
            "@com_google_impress//core/loader/provider/extensions:extension_meshopt_impl",
        ]
    else:
        deps = deps + [
            "@com_google_impress//core/loader/provider/extensions:extension_meshopt_noop",
        ]

    if "verification" in extensions:
        deps = deps + [
            "@com_google_impress//core/loader/provider/extensions:verification_impl",
        ]
    else:
        deps = deps + [
            "@com_google_impress//core/loader/provider/extensions:verification_noop",
        ]

    if "behavior" in extensions:
        deps = deps + [
            "@com_google_impress//core/loader/provider/extensions:extension_behavior_impl",
            "@com_google_impress//core/loader/provider/extensions:extension_interactivity_impl",
        ]
    else:
        deps = deps + [
            "@com_google_impress//core/loader/provider/extensions:extension_behavior_noop",
            "@com_google_impress//core/loader/provider/extensions:extension_interactivity_noop",
        ]

    if "webp" in extensions:
        deps = deps + [
            "@com_google_impress//core/image:webp_decode_image_noop",
        ]
    else:
        deps = deps + [
            "@com_google_impress//core/image:webp_decode_image_noop",
        ]
    cc_library(
        name = name,
        alwayslink = True,
        copts = imp_copts(),
        deps = deps,
        linkopts = imp_linkopts(),
        linkstatic = True,
        **kwargs
    )
