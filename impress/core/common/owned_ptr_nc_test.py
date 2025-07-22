# Copyright 2025 Google LLC
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


class OwnedPtrNCTest(googletest.TestCase):
  """Negative compilation tests for OwnedPtr and BorrowedPtr."""

  def testCompilerErrors(self):
    # Defines a list of test specs, where each element is a tuple
    # (test name, list of regexes for matching the compiler errors).
    test_specs = [
        ('IMPLICIT_DOWNCAST_COPY',
         [r'no viable conversion']),
        ('IMPLICIT_DOWNCAST_MOVE',
         [r'no viable conversion']),
        ('NONE', None),
    ]

    fake_target_util.AssertCcCompilerErrors(
        test_case=self,
        fake_target_path='google3/third_party/impress/core/common/owned_ptr_nc',
        target_name='owned_ptr_nc.o',
        test_specs=test_specs,
    )


if __name__ == '__main__':
  googletest.main()
