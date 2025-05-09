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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_BINDINGS_OBJECT_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_BINDINGS_OBJECT_H_

namespace imp {

// Superclass for all API bindings objects that have their native handle owned
// by the Java layer. This common base class is used to allow for a single call
// for common operations on all API bindings objects, such as the delete method.
class BindingsObject {
 public:
  BindingsObject() = default;
  virtual ~BindingsObject() = default;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_BINDINGS_OBJECT_H_
