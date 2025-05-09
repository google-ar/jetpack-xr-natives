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

"""Macro to merge jar files using the singlejar tool."""

def single_jar(name, jars):
    """
    Macro to combine jar files using the singlejar tool ((broken link)).

    Args:
      name: The base name for the generated rule and output JAR.
      jars: A list of labels pointing to jar files - typically the main output of android_library
    """
    output_jar_name = name + ".jar"
    input_jar_locations = ["$(location %s)" % jar for jar in jars]
    input_jar_string = " ".join(input_jar_locations)
    native.genrule(
        name = name,
        srcs = jars,
        outs = [output_jar_name],
        cmd = "$(location @bazel_tools//tools/jdk:singlejar) --output $@ --sources {jars}"
            .format(jars = input_jar_string),
        tools = ["@bazel_tools//tools/jdk:singlejar"],
    )
