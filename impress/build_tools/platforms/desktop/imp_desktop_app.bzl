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

"""Helper used by the imp_app rule to generate targets specific to the desktop platform."""

load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load(
    "@com_google_impress//build_tools:imp.bzl",
    "if_dev_runtime",
    "if_remote_desktop",
)
load(
    "@com_google_impress//build_tools/platforms:imp_app_common.bzl",
    "getTargetName",
)
load("@bazel_skylib//rules:build_test.bzl", "build_test")

def imp_desktop_app(
        name,
        impress_library,
        extra_linkopts = [],
        **kwargs):
    """Helper used by the imp_app rule to generate targets specific to the desktop platform.

    Args:
      name: Name of the imp_app rule. This helper will generate the target <name>_desktop.
      impress_library: The cc_library containing the native impress code.
      extra_linkopts: (Optional) additional linkopts to include in the generated cc_binary target.
      **kwargs: Additional keyword args that will be passed through to the cc_binary rule.
    """

    # Create two _desktop targets (one is testonly = True) and one _sandbox target.
    for target_type, testonly in (("desktop", False), ("desktop", True), ("sandbox", False)):
        # For desktop builds, simply combine the cross-platform code with the
        # desktop platform code in a c++ binary application.
        platform_name = name + "_" + target_type
        cc_binary(
            name = getTargetName(platform_name, testonly = testonly),
            linkopts = if_remote_desktop([
                # NOTE: This will cause the desktop binary to use the system linker and stop
                # using the libc from GRTE which can cause linking errors when dynamically
                # loading system libraries.
                #
                # This is required to support rendering with VirtualGL, which allows us to view
                # desktop apps over Chrome Remote Desktop.
                #
                # This means we are fully in unsupported territory but this seems to be the norm
                # for desktop applications in google3.
                #
                # See (broken link) and (broken link) for more context.
                "-Wl,--dynamic-linker=/lib64/ld-linux-x86-64.so.2",
            ]) + extra_linkopts,
            deps = if_dev_runtime(["@com_google_impress//core/performance:memory_stats"]) + [
                impress_library,
                Label("@com_google_impress//:platform_desktop"),
            ] + ([Label("@com_google_impress//core/view:app_sandbox_target")] if target_type == "sandbox" else []),
            testonly = testonly,
            **kwargs
        )
    platform_name = name + "_desktop"
    build_test(
        name = platform_name + "_build_test",
        targets = [getTargetName(platform_name, testonly = True)],
        tags = ["impress_app_build_test_desktop"],
    )
