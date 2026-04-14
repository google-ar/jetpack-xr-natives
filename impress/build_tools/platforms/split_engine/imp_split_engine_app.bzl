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

"""Helper functions for building cross platform apps using the Impress Framework."""

load(
    "@com_google_impress//build_tools/platforms:imp_app_common.bzl",
    "snakeCaseToCamelCase",
)
load(
    "@com_google_impress//build_tools/platforms/android:imp_generate_android_binary.bzl",
    "imp_generate_android_binary",
)

def imp_split_engine_app(
        name,
        jni_library,
        override_manifest,
        override_activity,
        package,
        activity_name,
        manifest_values,
        assets,
        resource_files,
        java_deps,
        native_lib_deps,
        nocompress_extensions = None,
        proguard_generate_mapping = None,
        proguard_specs = None,
        tags = None):
    """Helper used by the imp_app rule to generate targets specific to Split Engine.

    Args:
      name: Name of the imp_app rule. This helper will generate the target <name>_xr.
      jni_library: The jni library containing the native code for the Impress app.
      override_manifest: The manifest of the Xr app. Specify none to use a default manifest.
      override_activity: The manifest activity the Xr app. Specify none to use a default activity.
      package: The package of the Xr app. Specify none to use a default package generated from the name.
      activity_name: The name of the activity. Specify none to use a default name generated from the name.
      manifest_values: The manifest values merged into the manifest.
      assets: Assets passed through to the android_binary rule.
      resource_files: App resources for the resource_files attribute of android_binary.
      java_deps: Java dependencies passed through to the android_binary rule.
      native_lib_deps: Additional native .so deps to include in the Android apk.
      nocompress_extensions: (Optional) A list of file extensions that should not be
        compressed in the Android apk.
      proguard_generate_mapping: (Optional) Whether to generate a Proguard mapping file.
      proguard_specs: (Optional) A list of Proguard specs to use.
      tags: (Optional) tags to use on the android_library rule.
    """

    if not activity_name:
        activity_name = snakeCaseToCamelCase(name) + "SplitEngineActivity"

    manifest = Label("@com_google_impress//java/com/google/ar/imp/app/splitengine:AndroidManifest.xml")
    if override_manifest:
        manifest = override_manifest

    imp_generate_android_binary(
        name = name,
        platform_name = "split_engine",
        jni_library = jni_library,
        manifest = manifest,
        base_generated_activity = Label("@com_google_impress//java/com/google/ar/imp/app:basic_imp_split_engine_activity"),
        activity_name = activity_name,
        override_activity = override_activity,
        package = package,
        manifest_values = manifest_values,
        assets = assets,
        resource_files = resource_files,
        java_deps = java_deps,
        native_lib_deps = native_lib_deps,
        nocompress_extensions = nocompress_extensions,
        proguard_generate_mapping = proguard_generate_mapping,
        proguard_specs = proguard_specs if proguard_specs else [],
        # Ensure the split_engine targets get built with the correct config and android sdk.
        tags = ["split_engine"] + (tags if tags else []),
    )
