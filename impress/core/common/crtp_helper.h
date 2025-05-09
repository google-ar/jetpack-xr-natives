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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_CRTP_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_CRTP_HELPER_H_
namespace imp {

// Implements The Curiously Recurring Template Pattern (CRTP) helper class.
// For a complete understanding of uses and origin see:
// (broken link)/
// tldr: It's very similar to the decorator pattern but relies on templatization
// and inheritance vs virtual inheritance.
template <typename T>
struct CrtpHelper {
  T& GetUnderlying() { return static_cast<T&>(*this); }
  const T& GetUnderlying() const { return static_cast<const T&>(*this); }
};
}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_CRTP_HELPER_H_
