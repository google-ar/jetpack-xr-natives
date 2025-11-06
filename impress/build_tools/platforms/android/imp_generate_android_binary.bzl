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

"""Helper used to generate android_binary targets for android based platforms."""

load(
    "@com_google_impress//build_tools/platforms:imp_app_common.bzl",
    "getTargetName",
)
load(
    "@com_google_impress//core/view/platforms/android:generate_activity.bzl",
    "generate_activity",
)
load("@build_bazel_rules_android//android:rules.bzl", "android_binary", "android_library")
load("@bazel_skylib//rules:build_test.bzl", "build_test")

IMP_ANDROID_DEPS = [
    Label("//java/com/google/ar/imp/view:surface_view_no_jni"),
    
]

RESOURCES_DEPS = [
    "@maven//:androidx_appcompat_appcompat",  # Required for Impress' app theme that extends Theme.AppCompat.
]

def imp_generate_android_binary(
        name,
        platform_name,
        jni_library,
        manifest,
        base_generated_activity,
        activity_name,
        override_activity,
        package,
        manifest_values,
        assets,
        resource_files,
        java_deps,
        native_lib_deps,
        multidex,
        jni_binary_name = "",
        nocompress_extensions = None,
        proguard_generate_mapping = None,
        proguard_specs = None,
        tags = None):
    """Helper used by the imp_app rule to generate targets for platforms based on android.

    Args:
      name: Name of the imp_app rule. This helper will generate the target <name>_<platform_name>
      platform_name: Name of the platform used for generating the activity & the app.
      jni_library: The jni library containing the native code for the Impress app
      manifest: The manifest of the app. Specify none to use a default manifest.
      base_generated_activity: Base class that the automatically generated activity will subclass.
      activity_name: The name of the activity. Specify none to use a default name generated from the name.
      override_activity: The manifest activity the app. Specify none to use a default activity.
      package: The package of the app. Specify none to use a default package generated from the name.
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
      proguard_generate_mapping: (Optional) Whether to generate a Proguard mapping file.
      proguard_specs: (Optional) A list of Proguard specs to use.
      tags: (Optional) tags to use on the android_library rule.
    """

    if not package:
        package = "com.google.ar.imp.samples." + name

    full_platform_name = name + "_" + platform_name

    # For android, each app needs its own named activity. We simplify
    # this by generating a very simple activity to extend the activity provided by
    # base_activity.
    if (override_activity):
        activity_deps = [override_activity]
    else:
        generated_activity_name = full_platform_name + "_activity"
        generate_activity(
            name = generated_activity_name,
            activity_name = activity_name,
            base_activity_lib = base_generated_activity,
            package = package,
        )
        activity_deps = [":" + generated_activity_name]

    manifest_values_copy = dict(manifest_values)
    manifest_values_copy.update({
        "androidPackage": package,
        "appName": name,
        "activityName": activity_name,
        "binaryName": jni_binary_name,
    })

    for testonly in (False, True):
        # Create the actual Android APK.
        android_binary(
            name = getTargetName(full_platform_name, testonly = testonly),
            testonly = testonly,
            custom_package = package,
            manifest = manifest,
            assets = assets,
            tags = tags if tags else [],
            assets_dir = "",
            nocompress_extensions = nocompress_extensions if nocompress_extensions else [],
            manifest_values = manifest_values_copy,
            resource_files = [resource_files],
            deps = [":" + jni_library] + IMP_ANDROID_DEPS + java_deps + activity_deps + native_lib_deps + RESOURCES_DEPS,
            multidex = multidex,
            proguard_generate_mapping = proguard_generate_mapping if proguard_generate_mapping else False,
            proguard_specs = proguard_specs + ["@com_google_impress//build_tools/platforms/android:proguard.pgcfg"] if proguard_specs else [],
        )

    # Create an android library around all of the app code.  This is
    # separate from the actual app, and can be used for testing.
    # Note that when using this, the android_binary or android_library rule
    # will need to specify the 'appName' and 'activityName' manifest_values.
    # See third_party/impress/javatests/com/google/ar/imp/samples/simple/test/BUILD
    android_library(
        name = full_platform_name + "_lib",
        custom_package = package,
        exports_manifest = True,
        tags = tags if tags else [],
        manifest = manifest,
        resource_files = [resource_files],
        deps = [":" + jni_library] + IMP_ANDROID_DEPS + java_deps + activity_deps + native_lib_deps + RESOURCES_DEPS,
    )

    build_test(
        name = full_platform_name + "_build_test",
        targets = [getTargetName(full_platform_name, testonly = True)],
        tags = ["impress_app_build_test_" + platform_name] + (tags if tags else []),
    )
