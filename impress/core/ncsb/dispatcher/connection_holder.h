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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_CONNECTION_HOLDER_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_CONNECTION_HOLDER_H_

#include "core/ncsb/dispatcher/connection_id.h"

namespace imp {

// An interface to allow Event to call Dispatcher::EventHandlerMap.Disconnect().
// Don't use this class for anything external to the Dispatcher system.
class ConnectionHolder {
 public:
  virtual ~ConnectionHolder() {}
  virtual void Disconnect(ConnectionId connection_id) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_CONNECTION_HOLDER_H_
