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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_SYSTEM_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_SYSTEM_H_

#include "core/common/rememberer.h"
#include "filament/libs/utils/include/utils/Entity.h"

namespace imp {

class BaseView;

class ComponentManager;

// Base class for all systems. A system is an object that there should only be
// one of per-view that is used to perform operations on a batch of components.
//
// Systems may access the ComponentManager to access the components of various
// types to perform operations on them.
//
// Systems also act as a Rememberer, so they can be used with Future::KeptBy and
// as an owner in Connect methods on Dispatcher, Component, and Node.
class System : public imp::Rememberer {
 public:
  System() = delete;
  explicit System(BaseView* view);
  virtual ~System();

 protected:
  BaseView& GetView();

  // Returns the component manager owned by the view.
  ComponentManager& GetComponentManager();

 private:
  BaseView* view_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_SYSTEM_H_
