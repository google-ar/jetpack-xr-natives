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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_PASS_KEY_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_PASS_KEY_H_

namespace imp {

// "PassKey" pattern helper class, making something callable by a specific type.
//
// Example:
// Inside a class, make a public member that is only callable by class Foo:
//
// struct Bar {
//   void OnlyForFoo(PassKey<Foo> key) { ... }
// }
//
// Invoke from within Foo with an empty initializer for Key:
//
// struct Foo {
//   void Blah() {
//     Bar bar;
//     bar.OnlyForFoo({});
//   }
// }
//
// Only Foo can call Bar::OnlyForFoo() as PassKey<Foo> is only constructable by
// Foo.
template <typename T>
class PassKey {
  friend T;
  PassKey() {}
  PassKey(PassKey const&) = default;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_PASS_KEY_H_
