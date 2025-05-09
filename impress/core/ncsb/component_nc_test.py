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

from google3.testing.pybase import fake_target_util
from google3.testing.pybase import googletest


class NodeComponentNCTest(googletest.TestCase):
  """Negative compilation tests for Component."""

  def testCompilerErrors(self):
    # Defines a list of test specs, where each element is a tuple
    # (test name, list of regexes for matching the compiler errors).
    test_specs = [
        ('NO_SETUP_COMPONENT',
         [r'too many arguments to function call, expected 0, have 1']),
        ('SETUP_NO_ARG_COMPONENT',
         [r'too many arguments to function call, expected 0, have 1']),
        ('SETUP_INT_ARG_COMPONENT', [
            r'too few arguments to function call, single argument '
            r'\'test_value\' was not specified'
        ]),
        ('CAST_UNRELATED_COMPONENTS',
         [r'Cannot convert ComponentHandle types, they are unrelated.']),
        ('INVALID_SETUP_RETURN_TYPE_COMPONENT', [
            r'Component Setup method must return either '
            r'imp::Future<absl::Status>, absl::Status, or void.'
        ]),
        ('UPDATE_PHASE_DEPENDENCY_MISMATCH_COMPONENT', [
            r'UpdateDependencies must all have the same UpdatePhase as this '
            r'updater.'
        ]),
        ('UPDATE_PHASE_DEPENDENT_MISMATCH_COMPONENT', [
            r'UpdateDependents must all have the same UpdatePhase as this '
            r'updater.'
        ]),
        ('SANITY', None),
    ]

    fake_target_util.AssertCcCompilerErrors(
        test_case=self,
        fake_target_path='google3/third_party/impress/core/ncsb/component_nc',
        target_name='component_nc.o',
        test_specs=test_specs,
    )


if __name__ == '__main__':
  googletest.main()
