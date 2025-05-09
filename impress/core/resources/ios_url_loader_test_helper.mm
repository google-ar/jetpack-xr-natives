// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#import "core/resources/ios_url_loader_test_helper.h"

#import <Foundation/Foundation.h>

#include <string>

namespace imp::resources {

void CreateTestFile(absl::string_view filename, std::string content) {
  NSString *ns_filename = @(filename.data());
  NSString *ns_content = @(content.c_str());
  NSData *file_contents = [ns_content dataUsingEncoding:NSUTF8StringEncoding];
  [[NSFileManager defaultManager] createFileAtPath:ns_filename
                                          contents:file_contents
                                        attributes:nil];
}

void RemoveTestFile(absl::string_view filename) {
  NSString *ns_filename = @(filename.data());
  [[NSFileManager defaultManager] removeItemAtPath:ns_filename error:nil];
}

}  // namespace imp::resources
