# Copyright 2025 Google LLC
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

"""Build rules for generating max_api_requirement.h files."""

load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cc_toolchain", "use_cc_toolchain")
load("@rules_cc//cc/common:cc_common.bzl", "cc_common")
load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")

def _snakecase(name):
    return name.replace(".", "_").replace("/", "_").replace("-", "_").replace("+", "_").replace("{", "_").replace("}", "_").split("_")

def _upcase(name):
    return "_".join([x.upper() for x in _snakecase(name) if len(x) > 0])

def _gen_max_api_requirement_impl(ctx):
    """Implementation of the max_requires_api rule."""
    tool_output = ctx.actions.declare_file(ctx.attr.name + "_out.txt")

    args = ctx.actions.args()
    args.add_all(ctx.files.srcs)
    ctx.actions.run_shell(
        outputs = [tool_output],
        inputs = ctx.files.srcs,
        tools = [ctx.executable._tool],
        mnemonic = "GetMaxApiRequirement",
        command = "{tool} {srcs} > {out}".format(
            tool = ctx.executable._tool.path,
            srcs = " ".join([s.path for s in ctx.files.srcs]),
            out = tool_output.path,
        ),
    )

    ctx.actions.run_shell(
        inputs = [ctx.file._template, tool_output],
        outputs = [ctx.outputs.out],
        mnemonic = "GenMaxApiRequirement",
        command = ("sed -e 's/{{UPCASE_PATH}}/{upcase_path}/g' {template} | " +
                   "sed -e \"s/{{max_api_requirement}}/$(cat {tool_output})/g\" > {output}").format(
            upcase_path = _upcase(ctx.outputs.out.short_path),
            template = ctx.file._template.path,
            tool_output = tool_output.path,
            output = ctx.outputs.out.path,
        ),
    )

    cc_toolchain = find_cc_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
    )
    compilation_context, _ = cc_common.compile(
        name = ctx.attr.name + "_cc",
        feature_configuration = feature_configuration,
        actions = ctx.actions,
        cc_toolchain = cc_toolchain,
        public_hdrs = [ctx.outputs.out],
    )

    return [
        CcInfo(compilation_context = compilation_context),
    ]

# A rule that takes a list of flatbuffer schema files and outputs a header file
# containing the maximum requires_api attribute value found in the schemas.
gen_max_api_requirement = rule(
    implementation = _gen_max_api_requirement_impl,
    attrs = {
        "srcs": attr.label_list(allow_files = [".fbs"], allow_empty = False),
        "out": attr.output(mandatory = True),
        "_tool": attr.label(
            cfg = "exec",
            executable = True,
            default = Label("@third_party//split_engine/build_tools:get_schemas_max_api_requirement"),
        ),
        "_template": attr.label(
            allow_single_file = True,
            default = Label("@third_party//split_engine/build_tools:max_api_requirement.h.tmpl"),
        ),
    },
    toolchains = use_cc_toolchain(),
    fragments = ["cpp"],
)
