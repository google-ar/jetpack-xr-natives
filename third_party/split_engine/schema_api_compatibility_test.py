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

"""Test to validate API compatibility invariants in Split Engine schemas.

API Compatibility Rules:
   - If a table, struct, union, or enum is marked (preview_api), all of its
     contained fields or members must be explicitly marked (preview_api
     or requires_api: N), and cannot be left unmarked (experimental).
  - A field or union member marked (requires_api: N) must ONLY cascade into
     referenced types that are also marked (requires_api: M) where M <= N.
     It CANNOT reference a type that is unmarked (experimental), preview_api,
     or marked with a higher API level (M > N).
   - A field or union member marked (preview_api) must ONLY cascade into
     referenced types that are marked (preview_api) or (requires_api: N).
     It CANNOT reference a type that is unmarked (experimental).

Ignored Artifacts:
- Synthetic union discriminant fields (*_type) inserted by flatc.
- Synthetic NONE members inserted into unions by flatc.
"""

import os
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

from absl.testing import absltest
from flatbuffers.reflection import BaseType
from flatbuffers.reflection import Schema

from google3.devtools.api.source import presubmit_stubby_service_pb2
from google3.devtools.api.source.presubmit_build_target.python import (
    presubmit_utils,
)
from google3.third_party.bazel_rules.rules_python.python.runfiles import (
    runfiles,
)

# Ensure flatbuffers reflection package submodules are resolvable.
_runfiles_instance = runfiles.Create()
if _runfiles_instance:
  _fb_dir = _runfiles_instance.Rlocation('google3/third_party/py/flatbuffers')
  if _fb_dir and os.path.exists(_fb_dir) and _fb_dir not in sys.path:
    sys.path.insert(0, _fb_dir)

ApiState = Tuple[str, Optional[int]]


def get_attributes(obj_or_field: Any) -> Dict[str, Any]:
  """Extracts key-value attributes from a reflection object or field."""
  attrs = {}
  for i in range(obj_or_field.AttributesLength()):
    kv = obj_or_field.Attributes(i)
    key = kv.Key().decode('utf-8')
    val = kv.Value()
    if val is not None and len(val) > 0:
      attrs[key] = val.decode('utf-8')
    else:
      attrs[key] = True
  return attrs


def get_api_state(attrs: Dict[str, Any]) -> ApiState:
  """Returns (state_mode, api_level) tuple based on attributes dict."""
  if 'requires_api' in attrs:
    val = attrs['requires_api']
    if isinstance(val, bool):
      return ('requires_api', 1)
    return ('requires_api', int(val))
  elif 'preview_api' in attrs:
    return ('preview_api', None)
  else:
    return ('experimental', None)


def get_short_name(full_name: Optional[str]) -> str:
  """Strips namespace prefix from a type or member name."""
  return full_name.split('.')[-1] if full_name else ''


def check_container_cascading(
    container_name: str,
    container_kind: str,
    container_state: ApiState,
    member_name: str,
    member_kind: str,
    member_state: ApiState,
) -> List[str]:
  """Validates API state cascading from a parent container to a member."""
  c_mode, _ = container_state
  m_mode, _ = member_state
  errors = []

  if c_mode == 'preview_api' and m_mode == 'experimental':
    errors.append(
        f"{container_kind} '{container_name}' (preview_api) contains"
        f" {member_kind} '{member_name}' which is experimental (unmarked)"
    )

  return errors


def check_type_compatibility(
    item_desc: str,
    item_state: ApiState,
    type_name: str,
    type_state: ApiState,
) -> List[str]:
  """Validates API state compatibility between a field/member and its type.

  Enforces: A requires_api: N element must only cascade into referenced types
  that are also requires_api: M with M <= N.

  Args:
    item_desc: Description of the field or union member.
    item_state: The API state (mode, level) of the item.
    type_name: The name of the referenced type.
    type_state: The API state (mode, level) of the referenced type.

  Returns:
    A list of error message strings describing any compatibility violations.
  """
  item_mode, item_val = item_state
  type_mode, type_val = type_state
  errors = []
  if item_mode == 'preview_api' and type_mode == 'experimental':
    errors.append(
        f"{item_desc} is marked 'preview_api' but its type '{type_name}' is"
        ' experimental (unmarked)'
    )
  elif item_mode == 'requires_api':
    if type_mode != 'requires_api':
      errors.append(
          f"{item_desc} is marked 'requires_api: {item_val}' but its type"
          f" '{type_name}' is {type_mode}"
      )
    elif type_val > item_val:
      errors.append(
          f"{item_desc} is marked 'requires_api: {item_val}' but its type"
          f" '{type_name}' requires higher level '{type_val}'"
      )
  return errors


class SchemaApiCompatibilityTest(absltest.TestCase):
  """Test suite for Split Engine schema API compatibility checks.

  This suite powers the `CheckSchemaApiCompatibility` presubmit check configured
  in `third_party/split_engine/METADATA`. It validates API level compatibility
  rules across Split Engine flatbuffer schemas and verifies that rule
  violations are correctly detected.
  """

  def _load_schema(
      self,
      rlocation_subpath: Optional[str] = None,
  ) -> Schema.Schema:
    if rlocation_subpath is None:
      rlocation_subpath = (
          'google3/third_party/split_engine/split_engine_schemas.bfbs'
      )
    rlocation_path = (
        _runfiles_instance.Rlocation(rlocation_subpath)
        if _runfiles_instance
        else None
    )
    if rlocation_path and os.path.exists(rlocation_path):
      bfbs_path = rlocation_path
    else:
      filename = os.path.basename(rlocation_subpath)
      candidates = [
          os.path.join('third_party/split_engine', filename),
          os.path.join('blaze-bin/third_party/split_engine', filename),
          os.path.join('blaze-genfiles/third_party/split_engine', filename),
      ]
      bfbs_path = next(
          (c for c in candidates if os.path.exists(c)), candidates[0]
      )

    if not os.path.exists(bfbs_path):
      raise FileNotFoundError(
          f'Compiled binary schema not found at {bfbs_path}'
      )

    with open(bfbs_path, 'rb') as f:
      buf = f.read()

    return Schema.Schema.GetRootAs(buf, 0)

  def _check_objects(self, schema: Schema.Schema) -> List[str]:
    """Validates tables and structs."""
    errors = []
    for i in range(schema.ObjectsLength()):
      obj = schema.Objects(i)
      name = get_short_name(obj.Name().decode('utf-8'))
      obj_state = get_api_state(get_attributes(obj))

      for j in range(obj.FieldsLength()):
        field = obj.Fields(j)
        f_name = field.Name().decode('utf-8')
        f_state = get_api_state(get_attributes(field))

        f_type = field.Type()
        base_type = f_type.BaseType()
        element_type = f_type.Element()

        # Skip synthetic union discriminant fields (*_type) added by flatc.
        if f_name.endswith('_type') and base_type in (
            BaseType.BaseType.UType,
            BaseType.BaseType.Byte,
            BaseType.BaseType.UByte,
        ):
          continue

        # Rule 1: Container -> Field cascading checks
        errors.extend(
            check_container_cascading(
                name, 'Type', obj_state, f_name, 'field', f_state
            )
        )

        # Rule 2: Field -> Referenced Type cascading checks (requires_api)
        idx = f_type.Index()
        ref_attrs = None
        ref_name = None

        if base_type == BaseType.BaseType.Obj or (
            base_type == BaseType.BaseType.Vector
            and element_type == BaseType.BaseType.Obj
        ):
          if 0 <= idx < schema.ObjectsLength():
            ref_obj = schema.Objects(idx)
            ref_name = get_short_name(ref_obj.Name().decode('utf-8'))
            ref_attrs = get_attributes(ref_obj)
        elif 0 <= idx < schema.EnumsLength():
          ref_enum = schema.Enums(idx)
          ref_name = get_short_name(ref_enum.Name().decode('utf-8'))
          ref_attrs = get_attributes(ref_enum)

        if ref_attrs is not None and ref_name is not None:
          type_state = get_api_state(ref_attrs)
          item_desc = f"Field '{f_name}' in '{name}'"
          errors.extend(
              check_type_compatibility(item_desc, f_state, ref_name, type_state)
          )

    return errors

  def _check_enums_and_unions(self, schema: Schema.Schema) -> List[str]:
    """Validates enums and unions."""
    errors = []
    for i in range(schema.EnumsLength()):
      enum_def = schema.Enums(i)
      name = get_short_name(enum_def.Name().decode('utf-8'))
      enum_state = get_api_state(get_attributes(enum_def))
      is_union = enum_def.IsUnion()

      for j in range(enum_def.ValuesLength()):
        val = enum_def.Values(j)
        m_name = val.Name().decode('utf-8')
        m_state = get_api_state(get_attributes(val))

        if is_union:
          # Skip synthetic NONE member added to unions by flatc.
          if m_name == 'NONE':
            continue

          errors.extend(
              check_container_cascading(
                  name, 'Union', enum_state, m_name, 'member', m_state
              )
          )

          union_type = val.UnionType()
          if (
              union_type is not None
              and union_type.BaseType() == BaseType.BaseType.Obj
          ):
            u_idx = union_type.Index()
            if 0 <= u_idx < schema.ObjectsLength():
              ref_obj = schema.Objects(u_idx)
              ref_name = get_short_name(ref_obj.Name().decode('utf-8'))
              type_state = get_api_state(get_attributes(ref_obj))
              item_desc = f"Union member '{m_name}' in '{name}'"
              errors.extend(
                  check_type_compatibility(
                      item_desc, m_state, ref_name, type_state
                  )
              )
        else:
          errors.extend(
              check_container_cascading(
                  name, 'Enum', enum_state, m_name, 'member', m_state
              )
          )

    return errors

  def test_api_compatibility(self):
    """Validates API compatibility rules on live Split Engine binary schemas.

    This is the primary presubmit test invoked when Split Engine schemas are
    modified.
    """
    schema = self._load_schema()
    errors = []
    errors.extend(self._check_objects(schema))
    errors.extend(self._check_enums_and_unions(schema))
    self.assertEmpty(errors, '\n'.join(errors))

  def test_detects_schema_api_compatibility_violations(self):
    """Verifies that the compatibility checker catches artificial violations.

    Runs validation logic against a synthetic test schema containing invalid
    API level cascading patterns to ensure the presubmit checker fails when
    expected.
    """
    schema = self._load_schema((
        'google3/third_party/split_engine/'
        'build_tools/test_data/api_compatibility_test_schema.bfbs'
    ))
    errors = []
    errors.extend(self._check_objects(schema))
    errors.extend(self._check_enums_and_unions(schema))

    expected_error_substrings = [
        (
            "Type 'PreviewParentWithUnmarkedField' (preview_api) contains field"
            " 'unmarked_field' which is experimental"
        ),
        (
            "Union 'PreviewUnionWithUnmarkedMember' (preview_api) contains"
            " member 'ValidTarget' which is experimental"
        ),
        (
            "Enum 'PreviewEnumWithUnmarkedMember' (preview_api) contains member"
            " 'UnmarkedValue' which is experimental"
        ),
        (
            "Field 'invalid_preview_field' in 'PreviewFieldWithUnmarkedType' is"
            " marked 'preview_api' but its type 'UnmarkedType' is experimental"
        ),
        (
            "Field 'invalid_low_level_field' in"
            " 'LowLevelFieldWithHighLevelType' is marked 'requires_api: 1' but"
            " its type 'HighLevelType' requires higher level '2'"
        ),
        (
            "Field 'invalid_unmarked_field' in 'LowLevelFieldWithUnmarkedType'"
            " is marked 'requires_api: 1' but its type 'UnmarkedType' is"
            ' experimental'
        ),
        (
            "Union member 'invalid_member' in 'UnionWithHighLevelMember' is"
            " marked 'requires_api: 1' but its type 'HighLevelType' requires"
            " higher level '2'"
        ),
        (
            "Field 'invalid_preview_type_field' in"
            " 'LowLevelFieldWithPreviewType' is marked 'requires_api: 1' but"
            " its type 'PreviewType' is preview_api"
        ),
        (
            "Union member 'invalid_preview_member' in 'UnionWithPreviewMember'"
            " is marked 'requires_api: 1' but its type 'PreviewType' is"
            ' preview_api'
        ),
    ]

    for expected in expected_error_substrings:
      self.assertTrue(
          any(expected in err for err in errors),
          f"Expected error matching '{expected}' not found in actual errors:\n"
          + '\n'.join(errors),
      )


def main(argv: Sequence[str]) -> None:
  del argv
  suite = SchemaApiCompatibilityTest()
  errors = []
  try:
    schema = suite._load_schema()
    errors.extend(suite._check_objects(schema))
    errors.extend(suite._check_enums_and_unions(schema))
  except Exception as e:  # pylint: disable=broad-exception-caught
    errors.append(f'Failed to validate schema: {e}')

  succeeded = not errors
  failure_message = '\n'.join(errors) if errors else ''

  response = presubmit_stubby_service_pb2.PresubmitResponse(
      succeeded=succeeded,
      failure_message=failure_message,
  )
  presubmit_utils.WriteResponse(response)


if __name__ == '__main__':
  main(sys.argv)
