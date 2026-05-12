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

"""
Blaze rule for building Imp style C++ protobuf library.

Lifted with variation from //net/proto2/contrib/equals_plugin/build_defs.bzl
"""

load("@rules_cc//cc/common:cc_common.bzl", "cc_common")
load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")
load("@bazel_skylib//lib:paths.bzl", "paths")
load("@com_google_impress//build_tools:path_tools.bzl", "rlocation_path")
load("@bazel_tools//tools/cpp:toolchain_utils.bzl", "find_cpp_toolchain", "use_cpp_toolchain")

def _rule(ctx):
    headers = []
    proto_info = ctx.attr.proto[ProtoInfo]
    for direct_source in proto_info.direct_sources:
        header = _output_file(ctx, ".imp.h", direct_source)
        headers.append(header)

    if len(headers) < 1:
        fail("proto doesn't directly reference any .proto files, nothing to generate")

    ctx.actions.run(
        executable = ctx.executable._protocol_compiler,
        outputs = headers,
        inputs = depset(
            direct = [proto_info.direct_descriptor_set],
            transitive = [proto_info.transitive_descriptor_sets],
        ),
        tools = [ctx.executable._code_generator],
        arguments = [
            "--plugin=protoc-gen-imp-cc=" + ctx.executable._code_generator.path,
            "--descriptor_set_in=" + ":".join([f.path for f in proto_info.transitive_descriptor_sets.to_list()]),
            "--imp-cc_out=" + _bin_dir(ctx),
        ] + [rlocation_path(ctx, f) for f in proto_info.direct_sources],
    )
    return _compile_and_link(ctx, headers)

# NOLINT: variable is exported (not unused)
imp_cc_proto_library = rule(
    implementation = _rule,
    fragments = ["cpp"],
    attrs = {
        "_cc_lib": attr.label(default = "@com_google_impress//core/proto:proto_common"),
        "_code_generator": attr.label(
            cfg = "exec",
            default = "@com_google_impress//core/proto:proto-gen-imp-cc",
            executable = True,
        ),
        "_protocol_compiler": attr.label(
            cfg = "exec",
            default = "@com_google_protobuf//:protoc",
            executable = True,
        ),
        "_additional_deps": attr.label_list(default = [
            "@com_google_absl//absl/types:variant",
            "@com_google_absl//absl/strings:cord",
            "@com_google_impress//core/common:optional_with_default",
            "@com_google_impress//core/proto:proto_traits",
        ]),
        "proto": attr.label(
            doc = "proto_library to generate C++ for",
            mandatory = True,
            providers = [ProtoInfo],
        ),
        "deps": attr.label_list(
            doc = "cc_library targets that generated code depends on",
        ),
        "editor_proto_libs": attr.label_list(default = ["@com_google_impress//core/proto:imp_editor_cc_proto"]),
        "stringify_proto_libs": attr.label_list(default = ["@com_google_impress//core/proto:proto_stringify"]),
    },
    provides = [CcInfo],
    toolchains = use_cpp_toolchain(),
    doc = """
Generate Imp style C++ code from proto_library.
    """,
)

def _output_file(ctx, new_extension, old_file):
    """Declares the output file for the compiled proto."""
    return ctx.actions.declare_file(paths.basename(old_file.path) + new_extension, sibling = old_file)

def _bin_dir(ctx):
    """Gets the bin dir (including external/<workspace> when Impress is an external dep)."""
    path = ctx.bin_dir.path
    if ctx.label.workspace_name and ctx.label.workspace_name != ctx.workspace_name:
        path += "/external/" + ctx.label.workspace_name
    return path

def _compile_and_link(ctx, headers):
    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features + ["parse_headers", "layering_check"],
    )

    external_libs = ctx.attr.deps + [
        ctx.attr._cc_lib,
    ] + ctx.attr._additional_deps + ctx.attr.editor_proto_libs + ctx.attr.stringify_proto_libs

    cc_info_providers = [
        lib[CcInfo]
        for lib in external_libs
        if CcInfo in lib
    ]
    compilation_contexts = [provider.compilation_context for provider in cc_info_providers]
    linking_contexts = [provider.linking_context for provider in cc_info_providers]
    name = "{}".format(ctx.label.name)

    (compilation_context, compilation_outputs) = cc_common.compile(
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        public_hdrs = headers,
        compilation_contexts = compilation_contexts,
        name = name,
        user_compile_flags = [
            # We don't use exceptions.
            "-fno-exceptions",

            # Disable mac-specific warnings in our FormatString wrapper.
            "-Wno-format-security",
            "-Os",
        ],
    )

    linking_context, linking_outputs = cc_common.create_linking_context_from_compilation_outputs(
        actions = ctx.actions,
        name = name,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        linking_contexts = linking_contexts,
        compilation_outputs = compilation_outputs,
    )
    return [
        DefaultInfo(files = depset(headers)),
        CcInfo(
            compilation_context = compilation_context,
            linking_context = linking_context,
        ),
    ]
