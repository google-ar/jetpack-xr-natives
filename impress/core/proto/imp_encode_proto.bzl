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
Blaze rule for building ISF from textproto.
"""

load("@bazel_skylib//lib:paths.bzl", "paths")
load("@mediapipe//mediapipe/framework:encode_binary_proto.bzl", "encode_binary_proto")

IMP_DEFAULT_PROTOS = [
    str(Label("@com_google_impress//core/view/framework:framework_proto")),
    str(Label("@com_google_impress//core/ncsb:ncsb_proto")),
    str(Label("@com_google_impress//core/scene_handles:scene_handles_proto")),
]

def imp_encode_proto(
        name,
        srcs,
        message_type,
        extension_override = None,
        deps = [],
        visibility = None):
    """Converts a list of protocol buffers in text format into Imp Scene Files.

    Args:
      name: The name of the build target.  Creates a filegroup of binary .isf.
      srcs: The textproto files.
      message_type: The protobuf message type to encode, i.e. imp.NodeData.
      extension_override: By default will use message_type.lower() but accepts
          an override (i.e. "isf").
      deps: The list of app level custom proto_library rules that the srcs use.
          Transitive dependencies are pulled in automatically.
      visibility: Standard visibility attribute.
    """
    if not extension_override:
        extension_override = message_type.lower()
    extension = "%s." + extension_override

    full_deps = depset(IMP_DEFAULT_PROTOS + deps).to_list()
    outs = []
    for src in srcs:
        basename = paths.split_extension(src)[0]
        out = extension % basename
        outs.append(out)

        encode_binary_proto(
            name = basename + "_encoded_proto",
            input = src,
            message_type = message_type,
            output = out,
            deps = full_deps,
        )

    native.filegroup(
        name = name,
        srcs = outs,
        visibility = visibility,
    )
