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

"""Function to generate a named wrapper around a given activity."""

load("@build_bazel_rules_android//android:rules.bzl", "android_library")

def path_to_package(s):
    package = s.package
    return ".".join(package[package.find("com"):].split("/"))

def path_to_name(s):
    name = s.name
    return "".join([x.capitalize() for x in name.split("_")])

def generate_activity(
        name,
        package,
        activity_name,
        base_activity_lib):
    """Generates a Java activity class for a simple app.

    This activity will be a minimal wrapper around the base activity in base_activity_lib.
    The output activity will be:

        base_activity_package = path_to_package(base_activity_lib)
        base_activity_name = path_to_name(base_activity_lib)

        package <package>;
        import <base_activity_package>.<base_activity_name>;
        public class <activity_name> extends <base_activity_name> {}

    This is needed so that multiple apps can be included into a single apk.

    Args:
      name: The name of the output android_library
      package: The package to create the activity in.
              ex: "com.google.ar.imp.samples.simple"
      activity_name: The name of the Activity (UpperCamel). ex: "SimpleActivity"
      base_activity_lib: The android_library containing the custom base activity.
          The snake case library name should match the name of the class when converted
          to camel case.
    """

    base_activity_package = path_to_package(base_activity_lib)
    base_activity_name = path_to_name(base_activity_lib)

    java_file = "%s.java" % activity_name
    native.genrule(
        name = "%s_java_activity" % name,
        srcs = [],
        outs = [java_file],
        cmd = (
            "echo -e \"package " + package + ";\n\" > $@" +
            "echo \"import " + base_activity_package + "." + base_activity_name + ";\n\" >> $@" +
            "echo \"public final class " + activity_name + " extends " + base_activity_name + " {}\" >> $@"
        ),
    )
    android_library(
        name = name,
        srcs = [java_file],
        custom_package = package,
        deps = [base_activity_lib],
    )
