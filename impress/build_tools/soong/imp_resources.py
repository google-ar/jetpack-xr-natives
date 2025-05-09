#!/usr/bin/env python3
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

"""imp_resources.py embeds data resources so they can be compiled into a library."""

import argparse
import os
import re
import subprocess
import tempfile


def _read_file(path):
  with open(path, "r", encoding="utf-8") as f:
    return f.read()


def _expand_template(template_path, replacement_dictionary, output_path):
  """Replaces parameters in a template file.

  Args:
    template_path: path of template file.
    replacement_dictionary: dictionary of replacements.
    output_path: path for file after replacements.
  """

  content = _read_file(template_path)
  n = len(content)
  i = 0
  ret = ""
  while i < n:
    finds = [
        (content.find(var, i), var) for var in replacement_dictionary.keys()
    ]
    first = [(ix, var) for ix, var in sorted(finds) if ix >= 0]
    if first:
      ix, var = first[0]
      ret += content[i:ix]
      ret += replacement_dictionary[var]
      i = ix + len(var)
    else:
      ret += content[i:]
      i = n

  with open(output_path, "w", encoding="utf-8") as f:
    f.write(ret)


def generate_header(output_location, params):
  """Outputs headers for generated resources."""
  resource_package_id = _camelcase(_remove_ext(params.header_path))
  resource_definition = (
      "constexpr ::imp::resources::ResourceDefinition "
      + "{resource}"
      + "({url_and_identifier_args},"
      + " &Register{package_id});"
  )
  resource_list = "\n".join([
      resource_definition.format(
          resource=_resource_identifier(src),
          url_and_identifier_args=_resource_url_and_identifier_args(src),
          package_id=resource_package_id,
      )
      for src in params.srcs
  ])

  _expand_template(
      template_path=params.resource_template,
      replacement_dictionary={
          "{UPCASE_PATH}": _upcase(params.header_path),
          "{name}": params.package_name,
          "{camel_name}": _camelcase(params.package_name),
          "{namespace}": params.namespace,
          "{base_url}": '""',
          "{resources}": resource_list,
          "{package_id}": _camelcase(_remove_ext(params.header_path)),
      },
      output_path=output_location,
  )


def generate_cc(output_location, params):
  """Outputs source files for generated resources.

  Args:
    output_location: location for generated source file.
    params: params.package_name is the build target this is for.
      params.header_path the relative path to header. for includes, ifdefs and
      the classname
  """
  _expand_template(
      template_path=params.resource_template,
      replacement_dictionary={
          "{header_directory}": _dirname(params.header_path),
          "{header_path}": params.header_path,
          "{name}": params.package_name,
          "{camel_name}": _camelcase(params.package_name),
          "{namespace}": params.namespace,
          "{package_id}": _camelcase(_remove_ext(params.header_path)),
      },
      output_path=output_location,
  )


def _generate_filewrapper_output(
    filewrapper_binary,
    header_path,
    filewrapper_targets,
    output_path,
):
  """Executes filewrapper to create source/headers from templates.

  Args:
    filewrapper_binary: Location of filewrapper binary.
    header_path: The relative path to header. Used to create classname and
      parameters to includes and ifdefs.
    filewrapper_targets: List of files to be processed by the filewrapper
      subprocess.
    output_path: Location to be used for filewrapper output

  Returns:
  """
  # The filewrapper subprocess creates .cc and .h at the same time, but Soong
  # requires header generation in a different rule than source generation. To
  # match Soong design, a temporary file is used and discarded when this
  # function is run.  If generating source an unused header will be discarded
  # and vice-versa.

  # with isn't used here because the underlying filewrapper subprocess does all
  # the file writing.
  # pylint: disable=consider-using-with
  tmp = tempfile.NamedTemporaryFile(delete=True)
  if output_path.endswith(".h"):
    filewrapper_output_h = output_path
    # cc output is ignored for this pass.
    filewrapper_output_cc = tmp.name
  else:
    filewrapper_output_cc = output_path
    # header output is ignored in this pass.
    filewrapper_output_h = tmp.name

  filewrapper_name = _basename(_remove_ext(output_path))
  filewrapper_dir_path = _dirname(header_path)
  filewrapper_namespace = _camelcase(_remove_ext(header_path))
  filewrapper_args = [
      filewrapper_binary,
      filewrapper_dir_path,
      filewrapper_name,
      filewrapper_namespace,
      filewrapper_output_h,
      filewrapper_output_cc,
      "",
  ] + filewrapper_targets
  return subprocess.run(args=filewrapper_args, check=True)


def _basename(p):
  """Basename calculated from a path."""
  return os.path.basename(p)


def _dirname(p):
  """Dirname calculated from a path."""
  return os.path.dirname(p) or "."


def snakecase(package_path_or_name):
  """Package or path name with '_' between each word."""
  return (
      package_path_or_name.replace(".", "_")
      .replace("/", "_")
      .replace("-", "_")
      .replace("+", "_")
  )


def _upcase(package_path_or_name):
  """Package or path name with each word in upper case."""
  return "_".join(
      [x.upper() for x in snakecase(package_path_or_name).split("_")]
  )


def _camelcase(package_path_or_name):
  """Path name with all words except the first one capitalized."""
  snakecase_path = snakecase(package_path_or_name)
  upcase_array = re.sub(r"(_)+", " ", snakecase_path).title().replace(" ", "")
  return "".join([upcase_array[0].lower(), upcase_array[1:]])


def _remove_ext(src_path):
  """Filename with the extension removed."""
  return os.path.splitext(src_path)[0]


def _resource_identifier(src_path):
  """Identifier to be used for this path."""
  return _camelcase("k_" + _basename(src_path))


def _resource_url_and_identifier_args(src_path):
  """URL is just the filename because this script doesn't support remote content."""

  # Filewrapper strips out any part of the path that precedes the first instance
  # of "bin/". This is used to make asset urls look nicer when working with
  # generated files.
  if "bin/" in src_path:
    _, _, src_path = src_path.partition("bin/")

  return '"' + src_path + '"'


if __name__ == "__main__":
  parser = argparse.ArgumentParser(
      prog="imp_embedded_resources.py",
      description="Embed data resources in c sources",
  )
  parser.add_argument("--header_path", required=True)
  parser.add_argument("--package_name", required=True)
  parser.add_argument("--namespace", required=True)
  parser.add_argument("--output", required=True)
  parser.add_argument("--filewrapper", nargs=2, required=True)
  parser.add_argument("srcs", nargs="*")
  parser.add_argument(
      "--resource_template",
      required=True,
  )
  args = parser.parse_args()

  output_filename = args.output

  _generate_filewrapper_output(
      filewrapper_binary=args.filewrapper[0],
      header_path=args.header_path,
      filewrapper_targets=args.srcs,
      output_path=args.filewrapper[1],
  )

  if args.filewrapper[1].endswith(".h"):
    generate_header(output_filename, args)
  else:
    generate_cc(output_filename, args)
