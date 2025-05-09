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

"""Helper functions for embedding a set of Imp resources.

Resource mappings to allow resources to be easily embedded into development builds
accessed via the imp resource manager from the appropriate local or cloud-hosted
location."""

load("@bazel_skylib//lib:paths.bzl", "paths")
load("@com_google_impress//build_tools:imp.bzl", "imp_google3_copts")
load("@com_google_impress//build_tools:path_tools.bzl", "rlocation_path")

def _snakecase(name):
    return name.replace(".", "_").replace("/", "_").replace("-", "_").replace("+", "_").split("_")

def _camelcase(name):
    return "".join([x[0].capitalize() + x[1:] for x in _snakecase(name)])

def _upcase(name):
    return "_".join([x.upper() for x in _snakecase(name)])

def _resource_identifier(ctx, r):
    return "k{0}".format(_camelcase(paths.basename(rlocation_path(ctx, r))))

def _resource_url_and_identifier_args(ctx, r, base_url, urls):
    short_path = rlocation_path(ctx, r)
    if ctx.attr.embed:
        return '"{0}"'.format(short_path)
    else:
        final_url = paths.join(base_url, urls[r] if r in urls else short_path)
        return '"{0}", "{1}"'.format(short_path, final_url)

def _create_filewrapper(ctx, package_id):
    filewrapper_h = ctx.actions.declare_file(
        "{}_filewrapper.h".format(ctx.attr.basename),
    )
    filewrapper_cc = ctx.actions.declare_file(
        "{}_filewrapper.cc".format(ctx.attr.basename),
    )

    filewrapper_args = ctx.actions.args()

    filewrapper_args.add(ctx.label.package)
    filewrapper_args.add("{}_filewrapper".format(ctx.attr.basename))
    filewrapper_args.add(package_id)
    filewrapper_args.add(filewrapper_h)
    filewrapper_args.add(filewrapper_cc)
    filewrapper_args.add("")
    filewrapper_args.add_all(ctx.files.srcs)

    ctx.actions.run(
        executable = ctx.executable._filewrapper,
        inputs = ctx.files.srcs,
        outputs = [filewrapper_h, filewrapper_cc],
        arguments = [filewrapper_args],
    )

    return filewrapper_h, filewrapper_cc

def _gen_resources_rule(ctx):
    srcs = []
    private_hdrs = []

    resource_h = ctx.outputs.out_header

    # Use the path of the resource header as the package id so that it is
    # unique even if other rules use the same name and namespace.
    package_id = _camelcase(rlocation_path(ctx, resource_h))[:-1]

    if ctx.attr.embed:
        filewrapper_h, filewrapper_cc = _create_filewrapper(ctx, package_id)
        srcs.append(filewrapper_cc)
        private_hdrs.append(filewrapper_h)

    urls = {
        target.files.to_list()[0]: url
        for (target, url) in ctx.attr.urls.items()
        if not ctx.attr.embed and len(target.files.to_list()) == 1
    }
    base_url = ctx.attr.base_url if not ctx.attr.embed else ""

    resource_h_template = ctx.file._resource_template_header
    resource_fmt = ("constexpr ::imp::resources::ResourceDefinition {resource}(" +
                    "{url_and_identifier_args}, &Register{package_id});")

    resources = "\n".join([
        resource_fmt.format(
            resource = _resource_identifier(ctx, r),
            url_and_identifier_args = _resource_url_and_identifier_args(ctx, r, base_url, urls),
            package_id = package_id,
        )
        for r in ctx.files.srcs
    ])

    ctx.actions.expand_template(
        template = resource_h_template,
        output = resource_h,
        substitutions = {
            "{UPCASE_PATH}": _upcase(resource_h.short_path),
            "{name}": ctx.attr.basename,
            "{camel_name}": _camelcase(ctx.attr.basename),
            "{namespace}": ctx.attr.namespace,
            "{base_url}": base_url or "",
            "{resources}": resources,
            "{package_id}": package_id,
        },
    )

    resource_cc = ctx.actions.declare_file("{}.cc".format(ctx.attr.basename))

    if ctx.attr.embed:
        ctx.actions.expand_template(
            template = ctx.file._resource_embedded_template_source,
            output = resource_cc,
            substitutions = {
                "{header_path}": resource_h.short_path,
                "{name}": ctx.attr.basename,
                "{camel_name}": _camelcase(ctx.attr.basename),
                "{namespace}": ctx.attr.namespace,
                "{package_id}": package_id,
            },
        )
    else:
        resources_array_fmt = ("constexpr std::array<const ::imp::resources::ResourceDefinition *" +
                               ", {num_sources}> kResources{{ {resources_list} }};")
        resources_list = ", ".join([
            paths.join("&{0}".format(_resource_identifier(ctx, r)))
            for r in ctx.files.srcs
        ])
        resources_array = resources_array_fmt.format(
            resources_list = resources_list,
            num_sources = len(ctx.files.srcs),
        )

        ctx.actions.expand_template(
            template = ctx.file._resource_remote_template_source,
            output = resource_cc,
            substitutions = {
                "{header_path}": resource_h.short_path,
                "{namespace}": ctx.attr.namespace,
                "{package_id}": package_id,
                "{resources_array}": resources_array,
            },
        )

    srcs.append(resource_cc)

    return [DefaultInfo(files = depset(srcs + private_hdrs + [resource_h]))]

_gen_resources = rule(
    attrs = {
        "out_header": attr.output(
            mandatory = True,
            doc = "The name of the header file to be output containing the resource constants.",
        ),
        "basename": attr.string(
            mandatory = True,
            doc = "Basename to use to generate generated file names.",
        ),
        "srcs": attr.label_list(
            allow_files = True,
            doc = "The data files to be included in the resource bundle.",
        ),
        "namespace": attr.string(
            doc = "The namespace where the resource constants will be defined",
        ),
        "base_url": attr.string(doc = "base url for remote resources"),
        "urls": attr.label_keyed_string_dict(
            allow_empty = True,
            allow_files = True,
            doc = "Mapping of src labels to URLs.",
        ),
        "embed": attr.bool(
            default = True,
            doc = "If true (default), embeds assets into binary.",
        ),
        "_filewrapper": attr.label(
            executable = True,
            cfg = "exec",
            allow_files = True,
            default = Label("@com_google_sandboxed_api//sandboxed_api/tools/filewrapper"),
        ),
        "_resource_template_header": attr.label(
            allow_single_file = True,
            default = Label("@com_google_impress//core/resources:resource_h.tmpl"),
        ),
        "_resource_embedded_template_source": attr.label(
            allow_single_file = True,
            default = Label("@com_google_impress//core/resources:resource_embedded_cc.tmpl"),
        ),
        "_resource_remote_template_source": attr.label(
            allow_single_file = True,
            default = Label("@com_google_impress//core/resources:resource_remote_cc.tmpl"),
        ),
    },
    implementation = _gen_resources_rule,
)

def imp_resources(
        out_header,
        name,
        namespace,
        srcs,
        base_url = "",
        urls = {},
        embed = True,
        **kwargs):
    """
A Helper that defines a set of imp resources.

The resource is a mapping from an authoritative location where a set of files is
available in google3, along with the related network-hosted location where
these resources will be available for production builds.

Outputs:
    A cc_library named name exporting a header file named <name>.h which defines
    constants corresponding to each entry of srcs called <namespace>::k<SrcFile>
    which can be used as input to ResourceManager::Open calls
init
Args:
    name: the target name used to depend on these resources.
    namespace: then namespace in which the resource constants will be defined.
    srcs: the list of resource files which should be embedded in development
      builds, and loaded from the cloud in production builds.  Must be locations
      under the current directory in google3.
    base_url: the base network url at which the given resource files can be
      loaded based on their relative paths in google3 or based on the urls map.
    urls: map of file labels to URLs.  If no URL is specified for a label, the
      URL will be derived from the path.
    visibility: visibility for the generated rules.
"""

    # First step: generate some C++ files to wrap the given ressources.
    _gen_resources(
        out_header = out_header,
        name = name + "_cc_files",
        basename = name,
        namespace = namespace,
        srcs = srcs,
        base_url = base_url,
        urls = urls,
        embed = embed,
        **kwargs
    )

    # Second step: Wrap the generate files in the cc_library like output.
    native.cc_library(
        name = name,
        srcs = [":" + name + "_cc_files"],
        hdrs = [out_header],
        copts = imp_google3_copts(),
        alwayslink = 1,
        deps = [
            Label("@com_google_impress//core/resources"),
        ],
        **kwargs
    )
