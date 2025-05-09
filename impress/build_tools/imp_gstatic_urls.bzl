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

"""A helper to generate platform-specific gstatic asset links."""

load("@com_google_impress//build_tools:generated_gstatic_assets.bzl", "GSTATIC_URL_LOOKUP")

def clean_dep(dep):
    return str(Label(dep))

def imp_gstatic_urls(sources, labels):
    # Forge-on-mac (e.g. for Linux-hosted iOS tests) runs via VMWare, so no Metal and no gstatic.
    return select({
        "@third_party//filament:android": {clean_dep(source): GSTATIC_URL_LOOKUP["android"].get(label, "") for (source, label) in zip(sources, labels)},
        "@third_party//filament:filament_uses_opengl_ios": {clean_dep(source): GSTATIC_URL_LOOKUP["ios_gles3"].get(label, "") for (source, label) in zip(sources, labels)},
        "@third_party//filament:filament_uses_metal_ios": {clean_dep(source): GSTATIC_URL_LOOKUP["ios_metal"].get(label, "") for (source, label) in zip(sources, labels)},
        "@third_party//filament:ios": {clean_dep(source): GSTATIC_URL_LOOKUP["ios_metal"].get(label, "") for (source, label) in zip(sources, labels)},
        # If building in Exoblaze, it's NextCode. Otherwise, it's the simulator.
        # Support both NextCode and iOS Simulator Tests without depending on a config flag
        # by switching based on Exoblaze.
        # TODO: Find a better way to do this.
        # "@third_party//filament:ios_x86_64": {clean_dep(source): GSTATIC_URL_LOOKUP["ios_metal" if is_exoblaze else "ios_gles3"].get(label, "") for (source, label) in zip(sources, labels)},
        "@com_google_impress//core:ios_simulator_linux": {clean_dep(source): GSTATIC_URL_LOOKUP["ios_gles3"].get(label, "") for (source, label) in zip(sources, labels)},
        "@com_google_impress//core:ios_simulator_linux_metal": {clean_dep(source): GSTATIC_URL_LOOKUP["ios_metal"].get(label, "") for (source, label) in zip(sources, labels)},
        "@com_google_impress//core:ios_simulator_mac": {clean_dep(source): GSTATIC_URL_LOOKUP["ios_metal"].get(label, "") for (source, label) in zip(sources, labels)},
        "@third_party//filament:filament_uses_opengl_ios_x86_64": {clean_dep(source): GSTATIC_URL_LOOKUP["ios_gles3"].get(label, "") for (source, label) in zip(sources, labels)},
        "@third_party//filament:filament_uses_metal_ios_x86_64": {clean_dep(source): GSTATIC_URL_LOOKUP["ios_metal"].get(label, "") for (source, label) in zip(sources, labels)},
        "//conditions:default": {},
    })
