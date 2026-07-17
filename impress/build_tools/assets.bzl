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

"""Helper functions for using assets with the Impress Framework."""

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@com_google_impress//core/resources:resources.bzl", "camelcase", "imp_resources")

def _gen_assets_js_impl(ctx):
    lines = ["if (!globalThis.imp) { globalThis.imp = {}; }"]
    for f in ctx.files.srcs:
        # Match the C++ constant naming logic: kCamelCase
        c_name = "k" + camelcase(f.basename)

        # For the value, use the basename as that's what seems expected by loadModel
        lines.append("imp.{} = '{}';".format(c_name, f.basename))

    ctx.actions.write(ctx.outputs.out, "\n".join(lines))

_gen_assets_js = rule(
    implementation = _gen_assets_js_impl,
    attrs = {
        "srcs": attr.label_list(allow_files = True),
    },
    outputs = {"out": "%{name}.js"},
)

def _gen_assets_map_impl(ctx):
    name_camel = camelcase(ctx.attr.basename)
    header_guard = "IMPRESS_ASSETS_MAP_{}_H_".format(name_camel.upper())

    h_lines = [
        "#ifndef {}".format(header_guard),
        "#define {}".format(header_guard),
        "",
        "#include <map>",
        "#include <string>",
        "#include \"third_party/impress/core/resources/resource_definition.h\"",
        "#include \"{}\"".format(ctx.attr.constants_header),
        "",
        "namespace {} {{".format(ctx.attr.namespace),
        "",
        "const std::map<std::string, const imp::resources::ResourceDefinition*>& Get{}Map();".format(name_camel),
        "",
        "}}  // namespace {}".format(ctx.attr.namespace),
        "",
        "#endif  // {}".format(header_guard),
    ]

    out_h = ctx.actions.declare_file(ctx.attr.out_h)
    out_cc = ctx.actions.declare_file(ctx.attr.out_cc)

    cc_lines = [
        "#include \"{}\"".format(out_h.short_path),
        "#include \"{}\"".format(ctx.attr.constants_header),
        "",
        "namespace {} {{".format(ctx.attr.namespace),
        "",
        "const std::map<std::string, const imp::resources::ResourceDefinition*>& Get{}Map() {{".format(name_camel),
        "  static const std::map<std::string, const imp::resources::ResourceDefinition*> kMap = {",
    ]

    for f in ctx.files.srcs:
        c_name = "k" + camelcase(f.basename)
        cc_lines.append("    {{\"{}\", &{}}},".format(f.basename, c_name))

    cc_lines.append("  };")
    cc_lines.append("  return kMap;")
    cc_lines.append("}")
    cc_lines.append("")
    cc_lines.append("}}  // namespace {}".format(ctx.attr.namespace))

    ctx.actions.write(out_h, "\n".join(h_lines))
    ctx.actions.write(out_cc, "\n".join(cc_lines))

    return [DefaultInfo(files = depset([out_h, out_cc]))]

_gen_assets_map = rule(
    implementation = _gen_assets_map_impl,
    attrs = {
        "srcs": attr.label_list(allow_files = True),
        "basename": attr.string(),
        "namespace": attr.string(),
        "constants_header": attr.string(),
        "out_h": attr.string(),
        "out_cc": attr.string(),
    },
)

def imp_assets_base_url():
    return select({
        "@third_party//filament:android": "https://www.gstatic.com/ar/core/viewer/android",
        "@third_party//filament:filament_uses_opengl_ios": "https://www.gstatic.com/ar/core/viewer/ios_gles3",
        "@third_party//filament:filament_uses_metal_ios": "https://www.gstatic.com/ar/core/viewer/ios_metal",
        "@third_party//filament:ios": "https://www.gstatic.com/ar/core/viewer/ios_metal",
        
        "@com_google_impress//core:ios_simulator_linux_metal": "https://www.gstatic.com/ar/core/viewer/ios_metal",
        
        "@third_party//filament:filament_uses_opengl_ios_x86_64": "https://www.gstatic.com/ar/core/viewer/ios_gles3",
        "@third_party//filament:filament_uses_metal_ios_x86_64": "https://www.gstatic.com/ar/core/viewer/ios_metal",
        "//conditions:default": "",
    })

def imp_assets(
        name,
        srcs = [],
        namespace = "",
        base_url = "",
        urls = {},
        embed = True,
        generate_js = False,
        **kwargs):
    """Defines a set of imp assets.

    Generates a header that defines constants for a set of assets that are
    used to load assets at runtime through the AssetManager.

    The assets are either embedded into the binary or downloaded depending on if the
    [embed] attribute is true. It is recommended to create a build-time switch (i.e.
    through a config_setting) to control if assets are embedded or downloaded. This
    allows assets to be downloaded in production builds but embedded in development
    builds for rapid iteration.

    For an example usage, see the following files:
      @com_google_impress//samples/simple/BUILD
      @com_google_impress//samples/simple/simple_view.cc

    If generate_js is true, this rule also generates:

    1) An additional asset that represents the JS file to expose all the assets as
    identifiers inside the imp JS namespace. For example: for the asset Tiger.glb it
    will generate a JS constant imp.kTigerGlb. The name of the JS file constant will
    be {namespace}_identifiers.js and thus the constant will be exposed as
    {namespace}.k{name}IdentifiersJs
    2) A cc_library that exports a .h and .cc file that defines the matches of the
    JS string to actual native resources. The cc_library will be named {name}_js_map
    and the .h will be {name}_js_map.h and the .cc will be {name}_js_map.cc. The
    header file will expose a method to get the map of JS strings to native resources.
    The name of the function will be Get{name}JsMap().

    For example usage of the JS artifacts, see the following files:
      //third_party/impless/javascript/core/js/core/view/assets/BUILD
      //third_party/impless/javascript/core/js/core/view/assets/asset_manager_handler_test.cc

    Registering Assets:
        Assets must be registered before they can be loaded.

        Assets are automatically registered when passed into the AssetManager.

        However, they must be registered explicitly to load them by their string
        identifier. For example, when referencing a resource by string in a .isf
        file.

        To explicitly register assets, call the function
        foo_assets::RegisterPathToFoo() generated by the imp_assets rule.

        Registering the assets makes it possible to reference a resource by
        nothing but its string identifier and have it automatically switch between
        loading an embedded version of an asset and a remote version of an asset
        based on if the imp_assets were embedded in a build.

    See:
      third_party/impress/core/resources/resource_embedded_h.tmpl

    Outputs:
        A cc_library named <name> exporting a header file named <name>.h which defines
        constants corresponding to each entry of srcs called <namespace>::k<SrcFile>

    Args:
        name: The target name used to depend on these asset.
        namespace: The namespace in which the asset constants will be defined.
        srcs: The list of asset files which should be embedded. Must be locations
          under the current directory in google3.
        base_url: The base network url at which the given resource files can be
          loaded based on their relative paths in google3 or based on the urls map.
        urls: Map of file labels to URLs.  If no URL is specified for a label, the
          URL will be derived from the path.
        embed: If true (default), embeds assets into the binary. Otherwise, download assets based on the
           base_url and urls attributes.
        generate_js: If true, generates JavaScript identifiers file and C++ map library.
        **kwargs: Additional args passed through to the underlying imp_resources rule (i.e. visibility).
    """

    constants_header = name + ".h"

    if generate_js:
        identifiers_js_name = name + "_identifiers"

        # Generate a JS file with JS identifiers for native assets.
        _gen_assets_js(
            name = identifiers_js_name,
            srcs = srcs,
        )

        # Generate C++ map from the JS identifiers.
        map_gen_name = name + "_js_map_gen"
        _gen_assets_map(
            name = map_gen_name,
            srcs = srcs + [":" + identifiers_js_name],
            basename = name,
            namespace = namespace,
            constants_header = constants_header,
            out_h = name + "_js_map.h",
            out_cc = name + "_js_map.cc",
        )

        # TODO: Merge this into the same header as the main assets header.
        # Check (broken link) for a possible implementation proposal (it might be out of date).
        cc_library(
            name = name + "_js_map",
            srcs = [":" + map_gen_name],
            hdrs = [":" + map_gen_name],
            deps = [
                ":" + name,
                "@com_google_impress//core/resources",
            ],
            testonly = kwargs.get("testonly", False),
        )

        # In order for the conditional to work, the imp_resources rule must be duplicated.
        imp_resources(
            name = name,
            out_header = constants_header,
            srcs = srcs + [":" + identifiers_js_name],
            namespace = namespace,
            base_url = base_url,
            urls = urls,
            embed = embed,
            **kwargs
        )
    else:
        imp_resources(
            name = name,
            out_header = constants_header,
            srcs = srcs,
            namespace = namespace,
            base_url = base_url,
            urls = urls,
            embed = embed,
            **kwargs
        )
