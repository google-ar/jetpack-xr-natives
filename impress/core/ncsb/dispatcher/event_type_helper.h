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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_EVENT_TYPE_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_EVENT_TYPE_HELPER_H_

#include "core/common/type_traits.h"

namespace imp {

// Helper class for getting the HashValue from an Event type. Events can define
// their own custom static "HashValue GetEventTypeHash()" method to replace the
// default type_traits::kTypeHash<T> behavior.
class EventTypeHelper {
 public:
  // Return T::GetEventTypeHash() if defined, otherwise
  // type_traits::TypeHash<T>;
  template <typename T>
  static HashValue GetEventTypeHash() {
    if constexpr (kHasGetEventTypeHash<T>) {
      return T::GetEventTypeHash();
    } else {
      return type_traits::kTypeHash<T>;
    }
  }

 private:
  // This version of the function will only be defined if the class |T| has a
  // static "GetEventTypeHash" method.
  template <typename T,
            std::enable_if_t<
                std::is_same<HashValue, decltype(T::GetEventTypeHash())>::value,
                int> = 0>
  static constexpr bool HasGetEventTypeHash(int) {
    return true;
  }

  // If the above HasGetEventTypeHash isn't defined for the template parameters,
  // the compiler will default to calling this version.
  template <typename T>
  static constexpr bool HasGetEventTypeHash(...) {
    return false;
  }

  // True if the custom method is defined.
  template <typename T>
  static constexpr bool kHasGetEventTypeHash = HasGetEventTypeHash<T>(0);
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_EVENT_TYPE_HELPER_H_
