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

# Placeholder: load py_test

load("//build_tools:imp.bzl", "if_dev_runtime", "imp_google3_copts")
load(
    "//core/proto:imp_cc_proto_library.bzl",
    "imp_cc_proto_library",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

proto_library(
    name = "ncsb_proto",
    srcs = ["node_data.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
        "@com_google_protobuf//:any_proto",
    ],
)

imp_cc_proto_library(
    name = "ncsb_proto_cc",
    proto = ":ncsb_proto",
    deps = [
        "//core/math:math_proto_cc",
        "//core/proto:any_imp_proto",
    ],
)

cc_library(
    name = "node_flag",
    hdrs = [
        "node_flag.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//core/common:bit_flag",
    ],
    alwayslink = 1,
)

# Target for the impress framework's node-component-system-behavior API.
# Also includes the impress framework's dispatcher API, and some serialization functionality.
#
# TODO: Break out the dispatcher into its own library. This is non-trivial because
# there is a circular dependency between Node and Dispatcher.
cc_library(
    name = "ncsb",
    srcs = [
        "base_component_pool.cc",
        "base_node.cc",
        "component.cc",
        "component_manager.cc",
        "dispatcher/connection_owner.cc",
        "dispatcher/dispatcher.cc",
        "dispatcher/event.cc",
        "groups_manager.cc",
        "node.cc",
        "node_attachment_manager.cc",
        "node_controller.cc",
        "node_handle.cc",
        "path_manager.cc",
        "scene_metadata.cc",
        "system.cc",
        "update_system.cc",
    ],
    hdrs = [
        "base_component_pool.h",
        "base_node.h",
        "component.h",
        "component_handle.h",
        "component_id.h",
        "component_manager.h",
        "component_pool.h",
        "component_pool_with_updater.h",
        "component_system.h",
        "component_system_pool.h",
        "component_traits.h",
        "dispatcher/connection_holder.h",
        "dispatcher/connection_id.h",
        "dispatcher/connection_owner.h",
        "dispatcher/dispatcher.h",
        "dispatcher/event.h",
        "dispatcher/event_type_helper.h",
        "groups_manager.h",
        "isf_info.h",
        "node.h",
        "node_attachment_manager.h",
        "node_controller.h",
        "node_handle.h",
        "node_handle_message.h",
        "path_manager.h",
        "scene_metadata.h",
        "system.h",
        "update_id.h",
        "update_phase.h",
        "update_system.h",
        "updater_traits.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":ncsb_proto_cc",
        ":node_flag",
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/common",
        "//core/common:bit_flag",
        "//core/common:bit_vector",
        "//core/common:copyable_ptr",
        "//core/common:entity_absl_hasher",
        "//core/common:enum_indexed_array",
        "//core/common:filament_helpers",
        "//core/common:hash",
        "//core/common:holdable",
        "//core/common:invocable",
        "//core/common:owned_or_borrowed_ptr",
        "//core/common:owned_or_unowned_memory",
        "//core/common:owned_ptr",
        "//core/common:rememberer",
        "//core/common:robin_map",
        "//core/common:template_helpers",
        "//core/common:trace",
        "//core/common:type_traits",
        "//core/common:vector_helpers",
        "//core/geometry/shapes",
        "//core/graph:dependency_graph",
        "//core/lighting:environment_light",
        "//core/math",
        "//core/proto:any_imp_proto",
        "//core/proto:proto_common",
        "//core/proto:proto_differ",
        "//core/proto:proto_reader",
        "//core/proto:proto_writer",
        "//core/split_engine:split_engine_serializer",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "//core/view/async:future",
        "//core/view/utils:frame_time",
        "//core/view/utils:macros",
        "//core/view/utils:string_map",
        "//core/window:filament_host",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/base:no_destructor",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/container:fixed_array",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/container:inlined_vector",
        "@com_google_absl//absl/container:node_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:any",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:utils_hdrs",
        "@third_party//split_engine:split_engine_schema_version",
    ] + if_dev_runtime([
        "//core/editor:editor_info",
    ]),
    alwayslink = 1,
)

py_test(
    name = "component_nc_test",
    size = "large",
    srcs = ["component_nc_test.py"],
    data = ["component_nc"],
    tags = ["non_compile_test"],
    deps = [
        "//testing/pybase",
        "//testing/pybase:fake_target_util",
        "//third_party/py/google/protobuf:use_fast_cpp_protos",  # Automatically added (broken link)
    ],
)

cc_library(
    name = "test_component_native",
    hdrs = ["test_component_native.h"],
    deps = [":ncsb"],
    alwayslink = 1,
)

proto_library(
    name = "test_component_proto",
    srcs = ["test_component.proto"],
    deps = [
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "test_component_proto_cc",
    proto = ":test_component_proto",
    deps = [":test_component_native"],
)

# Proto library message for NodeHandleMessage, a common message type.
proto_library(
    name = "node_handle_message_proto",
    srcs = [
        "node_handle_message.proto",
    ],
    deps = [
        "//core/proto:imp_proto",
    ],
)

# Javascript flavor of node_handle_message generated code

# Imp flavor CC proto library for the NodeHandleMessage, a common message type.
imp_cc_proto_library(
    name = "node_handle_message_imp_proto",
    proto = ":node_handle_message_proto",
    deps = [":ncsb"],
)

java_lite_proto_library(
    name = "node_handle_message_java_proto_lite",
    deps = [":node_handle_message_proto"],
)

py_test(
    name = "dispatcher_nc_test",
    size = "large",
    srcs = ["dispatcher/dispatcher_nc_test.py"],
    data = ["dispatcher_nc"],
    tags = ["non_compile_test"],
    deps = [
        "//testing/pybase",
        "//testing/pybase:fake_target_util",
        "//third_party/py/google/protobuf:use_fast_cpp_protos",  # Automatically added (broken link)
    ],
)

proto_library(
    name = "test_event_proto",
    testonly = 1,
    srcs = ["dispatcher/test_event.proto"],
    deps = ["//core/proto:imp_proto"],
)

imp_cc_proto_library(
    name = "test_imp_event_proto",
    testonly = 1,
    proto = ":test_event_proto",
    deps = [
        ":ncsb",
        "//:testing_api",
        "@mediapipe//mediapipe/framework/port:gtest_main",
    ],
)
