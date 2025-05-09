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

load("@com_google_impress//core/proto:imp_encode_proto.bzl", "imp_encode_proto")

def imp_isf_data(
        name,
        srcs,
        deps = [],
        visibility = None):
    """Converts a list of protocol buffers in text format into Imp Scene Files.

    Args:
      name: The name of the build target.  Creates a filegroup of binary .isf.
      srcs: The textproto files.
      deps: The list of app level custom proto_library rules that the srcs use.
                  Transitive dependencies are pulled in automatically.
      visibility: Standard visibility attribute.
    """
    imp_encode_proto(name, srcs, "imp.NodeData", "isf", deps, visibility)
