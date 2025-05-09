/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_RESOURCES_CURL_URL_LOADER_H_
#define THIRD_PARTY_IMPRESS_CORE_RESOURCES_CURL_URL_LOADER_H_

#include <memory>

#include "absl/memory/memory.h"
#include "core/resources/url_loader.h"

namespace imp {
namespace resources {

// Creates a UrlLoader that uses curl as its underlying http request library.
std::unique_ptr<UrlLoader> CreateCurlLoader();

}  // namespace resources
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RESOURCES_CURL_URL_LOADER_H_
