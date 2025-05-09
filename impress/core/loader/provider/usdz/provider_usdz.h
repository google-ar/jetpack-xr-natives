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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_PROVIDER_USDZ_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_PROVIDER_USDZ_H_

#include <memory>

#include "core/loader/provider/details/usdz_provider.h"

namespace imp::loader::details::provider_usdz {

std::unique_ptr<UsdzProvider> CreateUsdzProvider();

}  // namespace imp::loader::details::provider_usdz

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_PROVIDER_USDZ_H_
