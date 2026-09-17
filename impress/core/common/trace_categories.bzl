# Copyright 2026 Google LLC
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
"""Trace categories for Impress."""

# Categories are defined as a dict of category name to a dict of metadata. In
# perfetto the category name for associated track events will be
# "impress.<category_name>".
# - groups: A list of groups that the category belongs to. If empty, the category
#   is not enabled by default. If non-empty, the category is enabled if any of
#   the groups are enabled. A groups inclusion in the compilation unit is
#   determined by the presence of the macro
#   IMP_TRACE_GROUP_ENABLE_<GROUP_NAME> which is set to 1 if the group is
#   enabled and 0 otherwise. "all" is an implicit group that enables all categories.
IMP_TRACE_CATEGORIES = {
    "default": {"groups": ["default"]},
    # Lower-priority default category; not emitted to perfetto by default.
    "default_detail": {"groups": []},
    # Category for the top level impress View.
    "view": {"groups": ["default"]},
    # Detail category for the top level impress View (not emitted by default).
    "view_detail": {"groups": []},
}

IMP_TRACE_CATEGORIES_LIST = list(IMP_TRACE_CATEGORIES.keys())

def _get_unique_groups():
    groups_dict = {}
    for category_dict in IMP_TRACE_CATEGORIES.values():
        for group in category_dict["groups"]:
            groups_dict[group] = True
    return list(groups_dict.keys())

IMP_TRACE_GROUPS = _get_unique_groups() + ["all"]

IMP_TRACE_CATEGORIES_COMMAND = """
cat << 'EOF' > $@
#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TRACE_CATEGORIES_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TRACE_CATEGORIES_H_

// The default category group is enabled by default.
#define IMP_TRACE_GROUP_ENABLE_default 1

""" + "\n".join(["#define IMP_VALID_CATEGORY_" + cat + " 1" for cat in IMP_TRACE_CATEGORIES_LIST]) + """

""" + "\n".join(["#ifndef IMP_TRACE_GROUP_ENABLE_" + group + "\n#define IMP_TRACE_GROUP_ENABLE_" + group + " 0\n#endif" for group in IMP_TRACE_GROUPS]) + """

""" + "\n".join([
    (
        "#ifndef IMP_TRACE_ENABLE_" + cat + "\n" +
        ("#if " + " || ".join(["IMP_TRACE_GROUP_ENABLE_" + g for g in (cat_dict["groups"] + ["all"])]) + "\n") +
        "#define IMP_TRACE_ENABLE_" + cat + " 1\n" +
        ("#else\n#define IMP_TRACE_ENABLE_" + cat + " 0\n#endif\n") +
        "#endif"
    )
    for cat, cat_dict in IMP_TRACE_CATEGORIES.items()
]) + """

""" + "\n".join([
    "#if IMP_TRACE_ENABLE_" + cat + "\n#define IMP_TRACE_IF_ENABLED_" + cat + "(...) __VA_ARGS__\n#else\n#define IMP_TRACE_IF_ENABLED_" + cat + "(...)\n#endif"
    for cat in IMP_TRACE_CATEGORIES_LIST
]) + """

#define IMP_PERFETTO_CATEGORIES \
""" + "  " + ", \\\n  ".join(["perfetto::Category(\"impress." + cat + "\")" for cat in IMP_TRACE_CATEGORIES_LIST]) + """

#endif // THIRD_PARTY_IMPRESS_CORE_COMMON_TRACE_CATEGORIES_H_
EOF
"""
