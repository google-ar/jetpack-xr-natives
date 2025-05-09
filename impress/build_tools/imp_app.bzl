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
    "@com_google_impress//build_tools:imp.bzl",
    "imp_default_jni_binary_name",
    "imp_default_jni_linkopts",
    "imp_default_split_engine_jni_binary_name",
    "imp_default_xr_jni_binary_name",
    "imp_google3_copts",
)
load(
    "@com_google_impress//build_tools/platforms/android:imp_android_app.bzl",
    "imp_android_app",
)
load(
    "@com_google_impress//build_tools/platforms/android:imp_app_resources.bzl",
    "imp_app_resources",
)
load(
    "@com_google_impress//build_tools/platforms/desktop:imp_desktop_app.bzl",
    "imp_desktop_app",
)
load(
    "@com_google_impress//build_tools/platforms/split_engine:imp_split_engine_app.bzl",
    "imp_split_engine_app",
)
load(
    "@com_google_impress//build_tools/platforms/xr:imp_xr_app.bzl",
    "imp_xr_app",
)

def imp_app(
        name,
        platforms = ["android"],
        srcs = [],
        hdrs = [],
        deps = [],
        tags = [],
        android_package = None,
        xr_package = None,
        split_engine_package = None,
        android_manifest = None,
        xr_manifest = None,
        split_engine_manifest = None,
        android_base_activity = None,
        xr_activity = None,
        split_engine_activity = None,
        android_activity_name = None,
        xr_activity_name = None,
        split_engine_activity_name = None,
        android_manifest_values = {},
        xr_manifest_values = {},
        split_engine_manifest_values = {},
        android_assets = [],
        android_lds = "//mobile/build:jni.lds",
        android_java_deps = [],
        android_native_lib_deps = [],
        android_jni_library_dep = None,
        multidex = "legacy",
        imp_impl = Label("@com_google_impress//:impl_full"),
        **_kwargs):
    """Generates Imp View app targets.

    TODO: Improve this documentation.

    This rule is useful to quickly build a cross platform standalone imp
    application.

    Args:
      name: The "base" name of the app binary targets. The actual name of the
        targets will be of the form "<basename>_<platform>".
      platforms: List of "platforms" to target.
             Must be: "android" for now as all other platforms are unsupported in Bazel.
      srcs: The C++ source files of the app.
      hdrs: The C++ headers files of the app.
      deps: The C++ dependencies for the app.  imp_assets rules should be
        included here.
      tags: Build tags to include in the app lib target.
      android_package: (Optional) The package of the android activity.
        Defaults to com.google.ar.imp.samples.<name>
      xr_package: (Optional) The package of the xr activity.
        Defaults to the android_package value.
      split_engine_package: (Optional) The package of the split engine activity.
        Defaults to the android_package value.
      android_manifest: (Optional) Custom AndroidManifest.xml file to be used
        for the android platform. Use this if the app needs extra permissions.
      xr_manifest: (Optional) Custom AndroidManifest.xml file to be used
        for the xr platform. Use this if the app needs extra permissions.
      split_engine_manifest: (Optional) Custom AndroidManifest.xml file to be used
        for the split engine app. Use this if the app needs extra permissions.
      android_base_activity: (Optional) The android_lib that provides a custom
        base activity class. The snake_case lib name should match the CamelCase
        activity name. If none provided, a default BasicImpActivity will be used
        to generate the activity.
      xr_activity: (Optional) The android_lib that provides a custom
        activity class for the xr platform. The snake_case lib name
        should match the CamelCase activity name. If none is provided, a default
        BasicImpXrActivity will be used to generate an activity.
      split_engine_activity: (Optional) The android_lib that provides a custom
        activity class for the split engine platform. The snake_case lib name
        should match the CamelCase activity name. If none is provided, a default
        BasicSplitEngineActivity will be used to generate an activity.
      android_activity_name: (Optional) The name of the android activity class.
      xr_activity_name: (Optional) The name of the xr activity class if the auto-generated name
        shouldn't be used.
      split_engine_activity_name: (Optional) The name of the xr activity class if
        the auto-generated name shouldn't be used.
      android_manifest_values: (Optional) Dictionary to use as the manifest_values
        when building the android binary.
      xr_manifest_values: (Optional) Dictionary to use as the manifest_values
        when building the android binary. Defaults to the android_manifest_values.
      split_engine_manifest_values: (Optional) Dictionary to use as the manifest_values
        when building the split engine binary. Defaults to the android_manifest_values.
      android_assets: (Optional) Android specifics assets passed into the android
        binary.
      android_lds: (Optional) The lds argument for the android_jni_library.
      android_java_deps: (Optional) Additional Java dependencies.
      android_native_lib_deps: (Optional) Additional native .so deps to include in the Android apk.
      android_jni_library_dep: (Optional) An existing jni library to use, rather than
        generating one inside this rule. Useful if you want to use both imp_app and
        also have a custom app setup.
      multidex: (Optional) Whether to allow the java dex to be split.
      imp_impl: (Optional) The implementation of optional impress features to use, i.e.
        either @com_google_impress//:impl_full or
        @com_google_impress//:impl_minimal. Defaults to full.
      **_kwargs: Prevents other kwargs that aren't supported in Bazel from erroring.
    """

    # The base c++ library, which includes all of the cross-platform code.
    #
    # alwaysLink is needed on iOS and MacOS to prevent the call to SetCreateViewFn in samples
    # from being stripped by the linker in iOS and MacOS builds. In production apps,
    # you can avoid making your entire app's library set to alwaysLink by separating
    # out the call to SetCreateViewFn in a separate small library from the rest of the app.
    # TODO: Find a better solution to this.
    native.cc_library(
        name = "lib",
        hdrs = hdrs,
        srcs = srcs,
        alwayslink = True,
        tags = tags,
        copts = imp_google3_copts(),
        deps = deps + [imp_impl, Label("@com_google_impress//:api")],
    )

    if "desktop" in platforms:
        imp_desktop_app(
            name = name,
            impress_library = "lib",
        )

    if "android" in platforms or "xr" in platforms or "split_engine" in platforms:
        app_resources_name = "app_resources"
        imp_app_resources(app_resources_name)

        if "android" in platforms:
            # Generate the android jni library if it wasn't explicitly specified.
            if not android_jni_library_dep:
                # Compile the cross-platform C++ code and the android platform code into
                # an android c++ library.
                native.cc_binary(
                    name = imp_default_jni_binary_name(),
                    linkopts = imp_default_jni_linkopts() + ["-llog"],
                    deps = [
                        Label("@com_google_impress//:platform_android"),
                    ] + [":lib"],
                    linkshared = True,
                )
                native.cc_import(
                    name = "jni",
                    shared_library = ":" + imp_default_jni_binary_name(),
                )
                android_jni_library_dep = "jni"

            # Generate the android targets.
            imp_android_app(
                name = name,
                jni_library = android_jni_library_dep,
                override_manifest = android_manifest,
                override_activity = android_base_activity,
                package = android_package,
                activity_name = android_activity_name,
                manifest_values = android_manifest_values,
                assets = android_assets,
                resource_files = ":" + app_resources_name,
                java_deps = android_java_deps,
                native_lib_deps = android_native_lib_deps,
                multidex = multidex,
            )

        if "xr" in platforms:
            if not xr_package:
                xr_package = android_package
            if not xr_manifest_values:
                xr_manifest_values = android_manifest_values

            # Used to inject Xr specific dependencies into the android_jni_library.
            # To avoid duplicate target names, this necessitates Xr builds having a different name
            # for the .so than other android builds, which is specified
            # by imp_default_xr_jni_binary_name().
            native.cc_binary(
                name = imp_default_xr_jni_binary_name(),
                linkopts = imp_default_jni_linkopts() + ["-llog"],
                deps = [
                    ":lib",
                    Label("@com_google_impress//:platform_android"),
                    Label("@com_google_impress//core/view/platforms/xr_android:xr_jni"),
                    imp_impl,
                ],
                linkshared = True,
            )
            native.cc_import(
                name = "xr_jni",
                shared_library = ":" + imp_default_xr_jni_binary_name(),
            )

            imp_xr_app(
                name = name,
                jni_library = "xr_jni",
                override_manifest = xr_manifest,
                override_activity = xr_activity,
                package = xr_package,
                activity_name = xr_activity_name,
                manifest_values = xr_manifest_values,
                assets = android_assets,
                resource_files = ":" + app_resources_name,
                java_deps = android_java_deps +
                            [Label("@com_google_impress//core/xr:openxr_loader_lib")],
                native_lib_deps = android_native_lib_deps,
                multidex = multidex,
            )

        if "split_engine" in platforms:
            if not split_engine_activity:
                split_engine_activity = android_base_activity
            if not split_engine_activity_name:
                split_engine_activity_name = android_activity_name
            if not split_engine_package:
                split_engine_package = android_package
            if not split_engine_manifest_values:
                split_engine_manifest_values = android_manifest_values

            # Used to inject split-engine-specific dependencies into the android_jni_library.
            # To avoid duplicate target names, this necessitates split engine builds having a
            # different name for the .so than other android builds, which is specified
            # by imp_default_xr_jni_binary_name().
            native.cc_binary(
                name = imp_default_split_engine_jni_binary_name(),
                linkopts = imp_default_jni_linkopts() + ["-llog"],
                deps = [
                    ":lib",
                    Label("@com_google_impress//:platform_android"),
                    Label("@com_google_impress//core/split_engine/android/view:split_engine_jni"),
                    imp_impl,
                ],
                linkshared = True,
            )

            native.cc_import(
                name = "split_engine_jni",
                shared_library = ":" + imp_default_split_engine_jni_binary_name(),
            )

            imp_split_engine_app(
                name = name,
                jni_library = "split_engine_jni",
                override_manifest = split_engine_manifest,
                override_activity = split_engine_activity,
                package = split_engine_package,
                activity_name = split_engine_activity_name,
                manifest_values = split_engine_manifest_values,
                assets = android_assets,
                resource_files = ":" + app_resources_name,
                java_deps = android_java_deps,
                native_lib_deps = android_native_lib_deps,
                multidex = multidex,
            )
