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

"""Helper used by the imp_app rule to generate targets specific to the android platform."""

load(
    "@com_google_impress//build_tools/platforms:imp_app_common.bzl",
    "snakeCaseToCamelCase",
)
load(
    "@com_google_impress//build_tools/platforms/android:imp_generate_android_binary.bzl",
    "imp_generate_android_binary",
)

def imp_android_app(
        name,
        jni_library,
        resource_files,
        multidex,
        jni_binary_name = "",
        override_manifest = None,
        override_activity = None,
        package = None,
        activity_name = None,
        manifest_values = {},
        assets = [],
        java_deps = [],
        native_lib_deps = [],
        nocompress_extensions = None,
        tags = None):
    """Helper used by the imp_app rule to generate targets specific to the android platform.

    Args:
      name: Name of the imp_app rule. This helper will generate the target <name>_android
      jni_library: The jni library containing the native code for the Impress app
      override_manifest: The manifest of the android app. Specify none to use a default manifest.
      override_activity: The manifest activity the android app. Specify none to use a default activity.
      package: The package of the android app. Specify none to use a default package generated from the name.
      activity_name: The name of the activity. Specify none to use a default name generated from the name.
      manifest_values: The manifest values merged into the manifest.
      assets: Assets passed through to the android_binary rule.
      resource_files: App resources for the resource_files attribute of android_binary.
      java_deps: Java dependencies passed through to the android_binary rule.
      native_lib_deps: Additional native .so deps to include in the Android apk.
      multidex: Multidex setting passed through to the android_binary rule.
      jni_binary_name: (Optional) The name of the binary generated from the jni library if it is
        overidden.
      nocompress_extensions: (Optional) A list of file extensions that should not be compressed in
        the Android apk.
      tags: (Optional) tags to use on the android_library rule.
    """
    if not activity_name:
        activity_name = snakeCaseToCamelCase(name) + "Activity"

    manifest = Label("//java/com/google/ar/imp/app:AndroidManifest.xml")
    if override_manifest:
        manifest = override_manifest

    imp_generate_android_binary(
        name = name,
        platform_name = "android",
        jni_library = jni_library,
        manifest = manifest,
        base_generated_activity = Label("//java/com/google/ar/imp/app:basic_imp_activity"),
        activity_name = activity_name,
        override_activity = override_activity,
        package = package,
        manifest_values = manifest_values,
        assets = assets,
        resource_files = resource_files,
        java_deps = java_deps,
        native_lib_deps = native_lib_deps,
        multidex = multidex,
        jni_binary_name = jni_binary_name,
        nocompress_extensions = nocompress_extensions,
        tags = tags,
    )
