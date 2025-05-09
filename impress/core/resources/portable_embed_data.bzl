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

"""a portable version of cc_embed_data"""

def portable_embed_data(
        name,
        srcs,
        flatten = False,
        visibility = None):
    """A lexan-compatible version of cc_embed_data.

    Args:
      name: name for the cc_library containing the accessors and data.
      srcs: the source files to embed into the library.
      flatten: Optional, if True, paths are stripped.
      visibility: Visibility of the generated cc_library.
    """
    flatten_flag = " --flatten" if flatten else ""

    # Emit a genrule which runs filewrapper and generates a pair of files.
    native.genrule(
        name = "%s_generator" % name,
        srcs = srcs,
        outs = [
            "%s.cc" % name,
            "%s.h" % name,
        ],
        cmd =
            "$(location @com_google_sandboxed_api//sandboxed_api/tools/filewrapper)" +
            " --out_cc=$(location %s.cc)" % name +
            " --out_h=$(location %s.h)" % name +
            flatten_flag +
            " --data_in_cc %s $(SRCS)" % name,
        tools = ["@com_google_sandboxed_api//sandboxed_api/tools/filewrapper"],
        visibility = visibility,
    )

    # Emit a cc_library with the generated files we just created
    native.cc_library(
        name = name,
        srcs = ["%s.cc" % name],
        hdrs = ["%s.h" % name],
        visibility = visibility,
    )
