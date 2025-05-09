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
  """Negative compilation tests for TypedSetVector()."""

  def testCompilerErrors(self):
    """Runs a list of tests to verify that marked code fails compilation."""

    # Defines a list of test specs, where each element is a tuple
    # (test name, list of regexes for matching the compiler errors).
    test_specs = [
        ('NO_MUTABLE_PROXY_FOR_CONST_VECTORS',
         [r'const fields cannot be assigned']),
        ('NO_IMPLICIT_MUTABLE_PROXY_FOR_CONST_VECTORS',
         [r"no viable overloaded '='"]),
        ('SANITY', None),
    ]

    fake_target_util.AssertCcCompilerErrors(
        self,
        'google3/third_party/impress/core/common/'
        'typed_set_vector_test_fake_binary',
        'typed_set_vector_test.o',
        test_specs,
    )


if __name__ == '__main__':
  googletest.main()
