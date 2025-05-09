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


class DispatcherNCTest(googletest.TestCase):
  """Negative compilation tests for Dispatcher."""

  def testCompilerErrors(self):
    # Defines a list of test specs, where each element is a tuple
    # (test name, list of regexes for matching the compiler errors).
    test_specs = [
        ('SEND_INVALID_EVENT', [
            r"no viable conversion from 'const imp::InvalidEvent' to 'const " +
            r"(imp::|)Event'"
        ]),
        ('WARN_UNUSED_RESULT', [r'warn_unused_result']),
        ('CONNECT_INVALID_EVENT',
         [r'Provided function must take a subclass of Event as a parameter.']),
        ('OWNER_VALUE', [
            r"requirement 'std::is_constructible<imp::ConnectionOwner, " +
            r"imp::MyClass>::value' was not satisfied"
        ]),
        ('OWNER_NOT_REMEMBERER', [
            r'Owner must contain a Remember method by either inheriting from ' +
            r'imp::Rememberer or defining a Remember method itself.'
        ]),
        ('SANITY', None),
    ]

    fake_target_util.AssertCcCompilerErrors(
        test_case=self,
        fake_target_path='google3/third_party/impress/core/ncsb/dispatcher_nc',
        target_name='dispatcher_nc.o',
        test_specs=test_specs,
    )


if __name__ == '__main__':
  googletest.main()
