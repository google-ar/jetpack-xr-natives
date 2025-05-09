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

"""Helpers to package the default icon and styles into the resources folder."""

def imp_app_resources(name):
    native.genrule(
        name = name + "_launcher",
        srcs = [
            Label("//java/com/google/ar/imp/app:ic_launcher.png"),
        ],
        outs = ["res/drawable/ic_launcher.png"],
        cmd = "cp $< $@",
    )
    native.genrule(
        name = name + "_styles",
        srcs = [
            Label("//java/com/google/ar/imp/app:styles.xml"),
        ],
        outs = ["res/values/styles.xml"],
        cmd = "cp $< $@",
    )
    native.filegroup(
        name = name,
        srcs = [":res/drawable/ic_launcher.png", ":res/values/styles.xml"],
    )
