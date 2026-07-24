# Copyright 2026 Google LLC
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

"""Build rules for generating flatbuffer_size_calculator_generated.h files."""

load("@rules_cc//cc:cc_library.bzl", "cc_library")

def _gen_flatbuffer_size_calculator_impl(ctx):
    """Implementation of the _gen_flatbuffer_size_calculator rule."""
    out_files = []
    for s in ctx.files.srcs:
        out_name = s.basename.replace(".fbs", "_size_calculator_generated.h")
        out_file = ctx.actions.declare_file(ctx.attr.out_prefix + out_name)
        out_files.append(out_file)

        args = ctx.actions.args()
        args.add("--out=" + out_file.path)
        if ctx.attr.scoped_enums:
            args.add("--scoped_enums")
        args.add(s.path)
        ctx.actions.run(
            outputs = [out_file],
            inputs = ctx.files.srcs,
            tools = [ctx.executable._tool],
            executable = ctx.executable._tool,
            arguments = [args],
            mnemonic = "GenFlatbufferSizeCalculator",
        )

    return [
        DefaultInfo(files = depset(out_files)),
    ]

_gen_flatbuffer_size_calculator = rule(
    implementation = _gen_flatbuffer_size_calculator_impl,
    attrs = {
        "srcs": attr.label_list(allow_files = [".fbs"], allow_empty = False),
        "out_prefix": attr.string(default = ""),
        "scoped_enums": attr.bool(default = False),
        "_tool": attr.label(
            cfg = "exec",
            executable = True,
            default = Label("@third_party//split_engine/build_tools:gen_flatbuffer_size_calculator"),
        ),
    },
)

def gen_flatbuffer_size_calculator(name, srcs, out_prefix = "", scoped_enums = False, deps = [], visibility = None):
    """Generates FlatBuffers size estimation headers and wraps them in a cc_library."""
    gen_name = name + "_gen"
    _gen_flatbuffer_size_calculator(
        name = gen_name,
        srcs = srcs,
        out_prefix = out_prefix,
        scoped_enums = scoped_enums,
    )

    cc_library(
        name = name,
        hdrs = [":" + gen_name],
        visibility = visibility,
        deps = deps + [
            "//third_party/flatbuffers:runtime_cc",
            "@third_party//split_engine:flatbuffer_size_calculator",
        ],
    )
