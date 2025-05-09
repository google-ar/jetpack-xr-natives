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

"""Generates end to end tests for the Impress scripting system"""

load(
    "//testing/web/build_defs:js.bzl",
    "js_web_test_suite",
)
load("//core/resources:portable_embed_data.bzl", "portable_embed_data")
load("@com_google_impress//build_tools:assets.bzl", "imp_assets")
load("@com_google_impress//build_tools:imp.bzl", "imp_test_copts")
load("@com_google_impress//testing:build_defs.bzl", "cc_test_on_android", "cc_test_on_ios")

def _concat_js_imp_assets(name, srcs, namespace):
    """Concatenates srcs in order and creates imp asset to load."""
    js_file_name = name + ".js"
    native.genrule(
        name = name + "_gen",
        testonly = True,
        srcs = srcs,
        outs = [js_file_name],
        cmd = "cat $(SRCS) > $@",
    )

    imp_assets(
        name = name,
        srcs = [js_file_name],
        namespace = namespace,
    )

    portable_embed_data(
        name = name + "_js",
        srcs = [
            js_file_name,
        ],
        flatten = True,
    )

def standalone_jasmine_test_script(name, srcs, namespace):
    """Outputs a script that will execute a Jasmine test when added to a browser window"""

    # Concat order is important:
    #  1. Load Jasmine lib
    #  2. Add jasmine and helper fns as globals
    #  3. Add test code
    #. 4. Execute tests
    _concat_js_imp_assets(
        name = name,
        srcs = [
            "//third_party/javascript/node_modules/jasmine_core:jasmine_code_jasmine",
            "@com_google_impress//javascript/testing:jasmine_window_bin.js",
        ] + srcs + ["@com_google_impress//javascript/testing:jasmine_boot_bin.js"],
        namespace = namespace,
    )

def _scripting_end_to_end_wasm_impl(name):
    """
    Generates end-to-end instrumented test for WASM.

    Args:
      name: Name of the generated target.
    """
    js_web_test_suite(
        name = name,
        test_libs = [
            "@com_google_impress//javascript/testing:wasm_e2e_launcher",
            "//testing/web/js/file_server",
            "//testing/web/js/web_test",
            "//third_party/javascript/node_modules/selenium_webdriver",
        ],
        browsers = [
            "//testing/web/browsers:chrome-linux",
            # TODO: Add additional browsers here.
        ],
        configs = [
            "//testing/web/configs:disable_web_security",
            "//testing/web/configs:enable_wtl_fileserver_cross_origin_isolation",
        ],
        # The example needs some files to run tests against.
        data = [
            # TODO: pass the test_model_resources dependency to this rule.
            "@com_google_impress//core/scripting/testing/wasm:e2e_wasm_webserver",
        ],
        visibility = ["//testing/web/js:__subpackages__"],
    )

def scripting_end_to_end_test(name, srcs = [], cc_deps = [], android_test_deps = [], ios_test_deps = [], java_deps = [], test_tags = []):
    """
    Generates end-to-end instrumented test for Android and iOS.

    Args:
      name: Name of the generated target.
      srcs: List of test srcs.
      cc_deps: List of cc deps.
      android_test_deps: List of android-specific test deps.
      ios_test_deps: List of ios-specific test deps.
      java_deps: List of java deps.
      test_tags: Tags for test.
    """
    cc_test_on_android(
        name = name + "_android_endtoend_test",
        srcs = srcs,
        cc_deps = android_test_deps + cc_deps,
        copts = imp_test_copts(),
        java_deps = java_deps,
        # TODO: Fix e2e test failure with compilation_mode=opt and remove noopt
        test_tags = ["noopt"] + test_tags,
    )

    cc_test_on_ios(
        name = name + "_ios_endtoend_test",
        srcs = srcs,
        copts = imp_test_copts(),
        test_host = "@bazel_tools//tools/build_defs/apple/testing:ios_default_host",
        test_tags = ["noopt"] + test_tags,
        deps = ios_test_deps + cc_deps,
    )

    _scripting_end_to_end_wasm_impl(
        name = name + "_wasm_endtoend_test",
    )
