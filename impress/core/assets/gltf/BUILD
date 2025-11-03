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

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:imp.bzl", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "gltf_audio_extension",
    srcs = [
        "gltf_audio_extension.cc",
    ],
    hdrs = [
        "gltf_audio_extension.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/async:future",
        "//core/audio",
        "//core/audio:audio_proto_cc",
        "//core/common:typed_ids",
        "//core/common:typed_set_vector",
        "//core/model",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@mediapipe//mediapipe/framework/port:status",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "gltf_behavior_extension",
    srcs = [
        "gltf_behavior_extension.cc",
    ],
    hdrs = [
        "gltf_behavior_extension.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/animation:property_animator",
        "//core/animation:property_animator_state_imp_cc_proto",
        "//core/assets/gltf/behavior:converted_graph",
        "//core/assets/gltf/behavior:node_converter_constants",
        "//core/assets/gltf/behavior:node_converters",
        "//core/async:future",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/math",
        "//core/math:arrays_proto_cc",
        "//core/model",
        "//core/model:model_data_details",
        "//core/ncsb",
        "//core/recipes:recipe_runner",
        "//core/recipes:recipe_runner_proto_cc",
        "//core/recipes/language:recipe_graph_cc",
        "//core/recipes/language:recipe_system",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:span",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "gltf_interactivity_extension",
    srcs = [
        "gltf_interactivity_extension.cc",
    ],
    hdrs = [
        "gltf_interactivity_extension.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/animation:property_animator",
        "//core/animation:property_animator_state_imp_cc_proto",
        "//core/assets/gltf/interactivity:converted_graph",
        "//core/assets/gltf/interactivity:node_converter_constants",
        "//core/assets/gltf/interactivity:node_converters",
        "//core/assets/gltf/interactivity/custom_statements",
        "//core/assets/gltf/interactivity/custom_statements:pointer_set",
        "//core/assets/gltf/interactivity/node_converters:utils",
        "//core/assets/gltf/interactivity/node_converters/animation:animation_node_converters",
        "//core/assets/gltf/interactivity/node_converters/debug:debug_node_converters",
        "//core/assets/gltf/interactivity/node_converters/event:event_node_converters",
        "//core/assets/gltf/interactivity/node_converters/flow:flow_node_converters",
        "//core/assets/gltf/interactivity/node_converters/math:math_node_converters",
        "//core/assets/gltf/interactivity/node_converters/pointer:pointer_node_converters",
        "//core/assets/gltf/interactivity/node_converters/variable:variable_node_converters",
        "//core/assets/gltf/object_model:pointer_parser",
        "//core/assets/gltf/object_model:property_pointer",
        "//core/assets/gltf/object_model/pointer_declarations:core_pointers",
        "//core/async:future",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/common:typed_set_vector",
        "//core/common:variant",
        "//core/math",
        "//core/math:arrays_proto_cc",
        "//core/model",
        "//core/model:model_data_details",
        "//core/ncsb",
        "//core/recipes:recipe_runner",
        "//core/recipes:recipe_runner_proto_cc",
        "//core/recipes/language:recipe_graph_cc",
        "//core/recipes/language:recipe_system",
        "//core/recipes/language:recipe_types_cc",
        "//core/recipes/language:recipe_utils",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:span",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)
