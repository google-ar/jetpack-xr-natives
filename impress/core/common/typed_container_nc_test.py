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

"""Negative compilation unit test for typed_container_helpers."""

from google3.testing.pybase import fake_target_util
from google3.testing.pybase import googletest


class TypedIdNCTest(googletest.TestCase):
  """Negative compilation tests for Typed IDs()."""

  def testCompilerErrors(self):
    """Runs a list of tests to verify that marked code fails compilation."""

    # Defines a list of test specs, where each element is a tuple
    # (test name, list of regexes for matching the compiler errors).
    test_specs = [
        ('NO_DEFAULT_CONSTRUCTOR_FOR_UNSIGNED', [r'no matching constructor']),
        (
            'NO_INTEGER_INDEXING_TYPED_VIEWS',
            [r'overload resolution selected deleted operator'],
        ),
        (
            'NO_INTEGER_INDEXING_TYPED_VECTORS',
            [r'overload resolution selected deleted operator'],
        ),
        ('RANGED_FOR_UNRELATED_ID_TYPED_VECTORS', [r'Wrong Id type']),
        ('ID_OF_UNRELATED_ID_TYPE', [r'no matching member function']),
        ('IDS_UNRELATED_ID_TYPE', [r'no viable conversion']),
        ('SANITY', None),
    ]

    fake_target_util.AssertCcCompilerErrors(
        self,
        'google3/third_party/impress/core/common/'
        'typed_container_test_nc',
        'typed_container_test.o',
        test_specs,
    )


if __name__ == '__main__':
  googletest.main()
