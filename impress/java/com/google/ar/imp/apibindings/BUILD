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

load("@build_bazel_rules_android//android:rules.bzl", "android_library")
load("@rules_kotlin//kotlin:jvm.bzl", "kt_jvm_library")
load("//build_tools/single_jar:single_jar.bzl", "single_jar")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

kt_jvm_library(
    name = "status",
    srcs = [
        "Status.kt",
    ],
)

android_library(
    name = "impress_api",
    srcs = [
        "IAssetAnimator.java",
        "IAssetLoader.java",
        "ImpressApi.java",
        "ImpressApiImpl.java",
        "Material.java",
        "Texture.java",
        "TextureSampler.java",
        "WaterMaterial.java",
    ],
    tags = ["included_in_archive=//java/com/google/ar/imp/apibindings:impress_no_native_lib_aar"],
    deps = [
        ":status",
        "//apibindings:impress_api_jni",
        "//java/com/google/ar/imp/view",
        "//java/com/google/ar/imp/view/splitengine",  # unuseddeps: keep
        "@maven//:androidx_annotation_annotation",
        "@maven//:androidx_concurrent_concurrent_futures",
        "@maven//:com_google_errorprone_error_prone_annotations",
        "@maven//:com_google_guava_guava",
        "@third_party//split_engine/java/com/google/androidxr/splitengine:split_engine_subspace_manager",  # unuseddeps: keep
        "@third_party//split_engine/java/com/google/androidxr/splitengine:subspace_node",  # unuseddeps: keep
    ],
)

# Generates a .jar containing Impress Java API bindings, Split Engine code, and Impress Java
# platform code. See THIRD_PARTY_NOTICES.txt for license info.
# Note: This JAR calls native code provided by the //apibindings:impress_api_jni
# target (.so) which must be available at runtime when using this .jar.
single_jar(
    name = "impress_no_native_lib_jar",
    # These .jar targets are the implicit outputs of android_library targets and, in Bazel,
    # kt_android_library targets.
    # TODO Modify single_jar to consume plain targets instead of implicitly-generated
    # .jar targets.
    # LINT.IfChange(jar_deps)
    jars = [
        "status.jar",
        "//java/com/google/ar/imp/view:libview.jar",
        "//java/com/google/ar/imp/view/splitengine:libsplitengine.jar",
        "//java/com/google/ar/imp/view/input:libinput_manager.jar",
        "//java/com/google/ar/imp/view:libimp_api_scuba.jar",
        "//java/com/google/ar/imp/view:libframe_scheduler.jar",
        "//java/com/google/ar/imp/view:libchoreographer_holder.jar",
        "//java/com/google/ar/imp/core/web:libfragment_host.jar",
        "@third_party//split_engine/java/com/google/androidxr/splitengine:libsplit_engine_subspace_manager.jar",
        "@third_party//split_engine/java/com/google/androidxr/splitengine:libsubspace_node.jar",
        "@third_party//split_engine/java/com/google/androidxr/splitengine:libsplit_engine_input_event.jar",
        "@third_party//split_engine/java/com/google/androidxr/splitengine:libscene_leash_node_factory.jar",
        ":libimpress_api.jar",
    ],
    # LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/apibindings:must_depend_jar)
)
