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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_EVENT_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_EVENT_H_

#include "core/ncsb/dispatcher/connection_holder.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/any.proto.imp.h"

namespace imp {

class EventWrapper;

// Base class for all events. Supports different methods of propagating the
// event throughout the hierarchy. The default propagation mode is kBubbleUp,
// the mode can be changed before sending.
//
// Important: All queries for the TypeHash of Events should use
// EventTypeHelper::GetEventTypeHash<T>() instead of GetTypeHash<T>().
class Event {
 public:
  // Determines how the event potentially propagates to other nodes.
  enum PropagationMode {
    // Sends to only the connections on the originating node.
    // The originating and the target node are the always the same.
    kNone,
    // Propagate to the connections on the originating node and it's ancestors
    // until Dispatcher::kAccept is returned.
    // Target node will always be the originating node or one of it's ancestors.
    kBubbleUp,
  };

  // Construct a default Event with kBubbleUp and no target or originating
  // node. The nodes are automatically filled in when the event is sent.
  Event();

  virtual ~Event() {}

  // Returns the propagation mode that Send was called with.
  PropagationMode GetPropagationMode() const;

  // Returns the node that the event was sent to.
  NodeHandle GetOriginatingNode() const;

  // Returns the node that the currently invoked function was Connect()ed with.
  NodeHandle GetTargetNode() const;

  // Sets the propagation mode, call before sending. Defaults to kBubbleUp.
  void SetPropagationMode(PropagationMode mode);

  // Serializes the event to an google::protobuf::Any.
  // Returns true if successful.
  virtual bool ToAny(google::protobuf::imp_proto::Any* any) const {
    return false;
  }

  // Disconnects the currently invoked function that the event was sent to.
  //
  // If the event is copied, it will disconnect the function the event was being
  // sent to when it was copied, or it will do nothing if it wasn't being sent
  // to an event. Note: should not be called after the Dispatcher is destroyed.
  void Disconnect() const;

  ConnectionId GetConnectionId() const { return connection_id_; }

 private:
  // Called automatically by Dispatcher when sent to set the originating node.
  void SetOriginatingNode(NodeHandle node);

  // Called automatically by Dispatcher when sent to set the current target
  // node.
  void SetTargetNode(NodeHandle node);

  // Called automatically by Dispatcher when sent to an event handler. Reset to
  // an empty function afterwards.
  void SetConnectionInfo(ConnectionHolder* connection_holder,
                         ConnectionId connection_id);

  PropagationMode mode_;
  NodeHandle originating_node_;
  NodeHandle target_node_;
  ConnectionHolder* connection_holder_;
  ConnectionId connection_id_;

  friend class Dispatcher;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_EVENT_H_
