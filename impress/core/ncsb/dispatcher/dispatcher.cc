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

#include "core/ncsb/dispatcher/dispatcher.h"

#include <cassert>
#include <cstddef>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "core/common/hash.h"
#include "core/common/invocable.h"
#include "core/common/robin_map.h"
#include "core/common/trace.h"
#include "core/ncsb/dispatcher/connection_holder.h"
#include "core/ncsb/dispatcher/connection_id.h"
#include "core/ncsb/dispatcher/connection_owner.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"

namespace imp {

// Stores a map of HashValue to EventHandlers that is used by the Dispatcher for
// sending events.
//
// The EventHandlers can be invoked via the Dispatch() function.  Adding and
// removing EventHandlers during Dispatch() is safely handled by storing the
// add/remove request in a "command queue" and processing the queue when the
// dispatch process is complete.  As a result, any EventHandler added during a
// Dispatch() will not be invoked.
//
// This class is not thread-safe.  All calls to an instance of this class must
// be done synchronously.
class DispatcherEventHandlerMap : public ConnectionHolder {
 public:
  using EventHandlerVariant = Dispatcher::EventHandlerVariant;
  using PropagationResult = Dispatcher::PropagationResult;
  using EventHandlerPropagationResult =
      Dispatcher::EventHandlerPropagationResult;
  using EventHandlerVoid = Dispatcher::EventHandlerVoid;

  DispatcherEventHandlerMap();

  // Associates an EventHandler with the specified event |type|.
  void Connect(NodeHandle node, HashValue type, ConnectionId id,
               const ConnectionOwner& owner, EventHandlerVariant fn);

  // Removes an EventHandler that matches the given parameters as best as
  // possible.
  void Disconnect(NodeHandle node, HashValue type, ConnectionId id,
                  const ConnectionOwner& owner);

  // Removes an EventHandler by id.
  void Disconnect(ConnectionId id) override;

  // Pass the |event| to all EventHandlers associated with the same HashValue as
  // the |event|. It propagates if configured to and returns the last NodeHandle
  // that was sent to.
  NodeHandle Dispatch(NodeHandle node, HashValue type, Event& event);
  PropagationResult DispatchImpl(NodeHandle node, HashValue type, Event& event);

  // Returns the number of active connections.
  size_t Size() const;

  // Returns the number of connections for an event of |type|.
  size_t GetHandlerCount(NodeHandle node, HashValue type) const;

  // Adds a forgetter to a handler.
  void SetForgetter(NodeHandle node, HashValue type, ConnectionId id,
                    Invocable<void()> forgetter);

 private:
  // Wraps an EventHandler with two extra "tags" (ConnectionId id and const
  // ConnectionOwner owner) that can be used to find specific EventHandler
  // instances.  Also associates a forgetter which is invoked when a handler
  // is disconnected.
  struct TaggedEventHandler {
    TaggedEventHandler(ConnectionId id, const ConnectionOwner& owner,
                       absl::optional<EventHandlerVariant> fn)
        : id(id), owner(owner), fn(std::move(fn)) {}
    ~TaggedEventHandler() {
      for (auto& forgetter : forgetters) {
        forgetter();
      }
    }

    TaggedEventHandler(const TaggedEventHandler&) = delete;
    TaggedEventHandler(TaggedEventHandler&&) noexcept = default;

    TaggedEventHandler& operator=(const TaggedEventHandler&) = delete;
    TaggedEventHandler& operator=(TaggedEventHandler&&) = delete;

    PropagationResult operator()(const Event& event) const {
      assert(fn);

      if (is_queued_for_disconnection) {
        return PropagationResult::kContinue;
      }

      if (absl::holds_alternative<EventHandlerPropagationResult>(*fn)) {
        return absl::get<EventHandlerPropagationResult>(*fn)(event);
      } else {
        absl::get<EventHandlerVoid> (*fn)(event);
        return PropagationResult::kContinue;
      }
    }

    explicit operator bool() const noexcept {
      if (!fn || is_queued_for_disconnection) {
        return false;
      }

      if (absl::holds_alternative<EventHandlerPropagationResult>(*fn)) {
        return static_cast<bool>(absl::get<EventHandlerPropagationResult>(*fn));
      } else {
        return static_cast<bool>(absl::get<EventHandlerVoid>(*fn));
      }
    }

    ConnectionId id;
    const ConnectionOwner owner;
    absl::optional<EventHandlerVariant> fn;
    std::vector<Invocable<void()>> forgetters;
    bool is_queued_for_disconnection = false;
  };

  // Deferred queue of add/remove commands while commands are in progress..
  using CommandQueue =
      std::vector<std::tuple<NodeHandle, HashValue, TaggedEventHandler>>;

  // Actually connects the EventHandler.
  void ConnectImpl(NodeHandle node, HashValue type,
                   TaggedEventHandler&& handler);

  // Disconnects the EventHandler.
  //
  // If |mark_queued_for_disconnection| is true, the handler will be marked as
  // queued for disconnection so that it will not be invoked, but it will not
  // actually be disconnected until the current operation is complete.
  void DisconnectImpl(NodeHandle node, HashValue type,
                      TaggedEventHandler&& handler,
                      bool mark_queued_for_disconnection = false);
  // Disconnect from just 1 map, and returns whether it is now empty.
  //
  // If |mark_queued_for_disconnection| is true, the handler will be marked as
  // queued for disconnection so that it will not be invoked, but it will not
  // actually be disconnected until the current operation is complete. In this
  // case, the function will return false because the map is not empty.
  bool DisconnectByType(
      std::unordered_multimap<HashValue, TaggedEventHandler>* map,
      HashValue type, const TaggedEventHandler& handler,
      bool mark_queued_for_disconnection);

  void DrainCommandQueue(CommandQueue& command_queue);

  // Counter for tracking Dispatch() calls.
  int dispatch_count_;

  // Counter for tracking Disconnect() calls.
  int disconnect_count_;

  // Deferred queue of add/remove commands for when Dispatch() is in progress.
  CommandQueue dispatch_command_queue_;

  // Deferred queue of add/remove commands for when Disconnect() is in progress.
  CommandQueue disconnect_command_queue_;

  // Map of registered handlers.
  RobinMap<NodeHandle, std::unordered_multimap<HashValue, TaggedEventHandler>>
      map_;

  // Map to store additional data by id to make disconnecting easier.
  RobinMap<ConnectionId, std::pair<NodeHandle, HashValue>> connections_by_id_;
};

Dispatcher::Dispatcher() { handlers_.reset(new EventHandlerMap()); }

Dispatcher::ScopedConnection Dispatcher::Connect(
    HashValue type, EventHandlerPropagationResult handler) {
  return ConnectVariant(type, EventHandlerVariant(std::move(handler)));
}

Dispatcher::ScopedConnection Dispatcher::Connect(HashValue type,
                                                 EventHandlerVoid handler) {
  return ConnectVariant(type, EventHandlerVariant(std::move(handler)));
}

Dispatcher::ScopedConnection Dispatcher::Connect(
    NodeHandle node, HashValue type, EventHandlerPropagationResult handler) {
  return ConnectVariant(node, type, EventHandlerVariant(std::move(handler)));
}

Dispatcher::ScopedConnection Dispatcher::Connect(NodeHandle node,
                                                 HashValue type,
                                                 EventHandlerVoid handler) {
  return ConnectVariant(node, type, EventHandlerVariant(std::move(handler)));
}

Dispatcher::ScopedConnection Dispatcher::ConnectVariant(
    HashValue type, EventHandlerVariant handler) {
  return ConnectVariant(NodeHandle(), type, std::move(handler));
}

Dispatcher::ScopedConnection Dispatcher::ConnectVariant(
    NodeHandle node, HashValue type, EventHandlerVariant handler) {
  assert(type != 0);
  return ConnectImpl(node, type, std::move(handler), ConnectionOwner());
}

void Dispatcher::Disconnect(NodeHandle node, HashValue type, ConnectionId id) {
  assert(id != kNullConnectionId && type != 0);
  handlers_->Disconnect(node, type, id, ConnectionOwner());
}

void Dispatcher::Disconnect(ConnectionId id) {
  assert(id != kNullConnectionId);
  handlers_->Disconnect(id);
}

void Dispatcher::DisconnectAll(const ConnectionOwner& owner) {
  // Calling Remove() should assert type != 0 or id != kNullConnectionId
  // unless setting them manually.
  handlers_->Disconnect(NodeHandle(), 0, kNullConnectionId, owner);
}

NodeHandle Dispatcher::SendImpl(NodeHandle node, HashValue type_hash,
                                const Event& event) {
  // Event can't be const so that it's possible for the dispatcher to set
  // the originating node, target node, connection info, etc.
  // TODO: Elminate this const_cast.
  Event& non_const_event = const_cast<Event&>(event);
  return handlers_->Dispatch(node, type_hash, non_const_event);
}

Dispatcher::Connection Dispatcher::ConnectImpl(NodeHandle node, HashValue type,
                                               EventHandlerVariant handler,
                                               const ConnectionOwner& owner) {
  // All callers for ConnectImpl() other than ConnectToAll() assert type !=
  // 0.
  const ConnectionId id = ++id_;
  handlers_->Connect(node, type, id, owner, std::move(handler));
  auto connection = Connection(handlers_, node, type, id);

  // If we are targetting a node and not being owned by that node, also cleanup
  // the connection when the target node is destroyed.
  if (!node.IsDefaultValue() && node != owner.GetNode()) {
    KeepAlive(node, connection);
  }
  return connection;
}

void Dispatcher::DisconnectImpl(NodeHandle node, HashValue type,
                                const ConnectionOwner& owner) {
  assert(type != 0);
  handlers_->Disconnect(node, type, kNullConnectionId, owner);
}

size_t Dispatcher::GetHandlerCount() const { return handlers_->Size(); }

size_t Dispatcher::GetHandlerCount(HashValue type) const {
  return GetHandlerCount(NodeHandle(), type);
}

size_t Dispatcher::GetHandlerCount(NodeHandle node, HashValue type) const {
  return handlers_->GetHandlerCount(node, type);
}

DispatcherConnection::DispatcherConnection()
    : type_(0), id_(kNullConnectionId), handlers_() {}

DispatcherConnection::DispatcherConnection(
    const DispatcherEventHandlerMapPtr& handlers, NodeHandle node,
    HashValue type, ConnectionId id)
    : node_(node), type_(type), id_(id), handlers_(handlers) {}

void DispatcherConnection::Disconnect() {
  // type_ is allowed to be null if this is the ConnectToAll connection.
  if (id_ == kNullConnectionId) {
    return;
  }
  if (auto handlers = handlers_.lock()) {
    handlers->Disconnect(node_, type_, id_, ConnectionOwner());
    handlers_.reset();
  }
}

ConnectionId DispatcherConnection::GetId() const { return id_; }
HashValue DispatcherConnection::GetTypeHash() const { return type_; }
NodeHandle DispatcherConnection::GetNode() const { return node_; }

void Dispatcher::SetForgetter(const Dispatcher::Connection& conn,
                              Invocable<void()> forgetter) {
  handlers_->SetForgetter(conn.GetNode(), conn.GetTypeHash(), conn.GetId(),
                          std::move(forgetter));
}

Dispatcher::ScopedConnection::ScopedConnection(ScopedConnection&& rhs)
    : connection_(std::move(rhs.connection_)) {
  rhs.connection_ = Connection();
}

Dispatcher::ScopedConnection& Dispatcher::ScopedConnection::operator=(
    ScopedConnection&& rhs) {
  if (this != &rhs) {
    Disconnect();
    connection_ = rhs.connection_;
    rhs.connection_ = Connection();
  }
  return *this;
}

Dispatcher::ScopedConnection::ScopedConnection(Connection c) : connection_(c) {}

Dispatcher::ScopedConnection::~ScopedConnection() { Disconnect(); }

void Dispatcher::ScopedConnection::Disconnect() { connection_.Disconnect(); }

DispatcherEventHandlerMap::DispatcherEventHandlerMap()
    : dispatch_count_(0), disconnect_count_(0) {}

void DispatcherEventHandlerMap::Connect(NodeHandle node, HashValue type,
                                        ConnectionId id,
                                        const ConnectionOwner& owner,
                                        EventHandlerVariant fn) {
  TaggedEventHandler handler(id, owner, std::move(fn));
  if (dispatch_count_ > 0) {
    dispatch_command_queue_.emplace_back(node, type, std::move(handler));
  } else if (disconnect_count_ > 0) {
    disconnect_command_queue_.emplace_back(node, type, std::move(handler));
  } else {
    ConnectImpl(node, type, std::move(handler));
  }
}

void DispatcherEventHandlerMap::Disconnect(NodeHandle node, HashValue type,
                                           ConnectionId id,
                                           const ConnectionOwner& owner) {
  TaggedEventHandler handler(id, owner, absl::nullopt);
  if (dispatch_count_ > 0) {
    // This marks the handler as queued for disconnection so that it will not
    // be invoked if the event is sent before the command queue is drained, but
    // it will not actually be disconnected until the current dispatch operation
    // is complete.
    //
    // An extra TaggedEventHandler is created here beacuse it's a move only type
    // & used to identify the handler within the map. This is safe, but should
    // be refactored fir code clarity. The TaggedEventHandler is used in an
    // overloaded way, both to store the handlers in a map, and to identify them
    // when looking them up. It should really only be used for the former case.
    // Using it for the latter is a bit confusing.
    // TODO: Refactor this.
    DisconnectImpl(node, type, TaggedEventHandler(id, owner, absl::nullopt),
                   true);

    dispatch_command_queue_.emplace_back(node, type, std::move(handler));

  } else if (disconnect_count_ > 0) {
    disconnect_command_queue_.emplace_back(node, type, std::move(handler));
  } else {
    DisconnectImpl(node, type, std::move(handler));
  }
}

void DispatcherEventHandlerMap::Disconnect(ConnectionId id) {
  auto iter = connections_by_id_.find(id);
  if (iter != connections_by_id_.end()) {
    Disconnect(iter->second.first, iter->second.second, id, ConnectionOwner());
  }
}

void DispatcherEventHandlerMap::ConnectImpl(NodeHandle node, HashValue type,
                                            TaggedEventHandler&& handler) {
  assert(handler.id != kNullConnectionId);
  assert(handler);
  connections_by_id_[handler.id] = {node, type};
  map_[node].emplace(type, std::move(handler));
}

bool DispatcherEventHandlerMap::DisconnectByType(
    std::unordered_multimap<HashValue, TaggedEventHandler>* map, HashValue type,
    const TaggedEventHandler& handler, bool mark_queued_for_disconnection) {
  // If DisconnectAll, check all nodes.
  auto range = std::make_pair(map->begin(), map->end());
  if (type != 0 || handler.id != kNullConnectionId) {
    range = map->equal_range(type);
  }

  if (handler.id != kNullConnectionId) {
    for (auto it = range.first; it != range.second; ++it) {
      if (it->second.id == handler.id) {
        connections_by_id_.erase(handler.id);
        if (mark_queued_for_disconnection) {
          it->second.is_queued_for_disconnection = true;
        } else {
          map->erase(it);
        }
        break;
      }
    }
  } else if (handler.owner.IsValid()) {
    for (auto it = range.first; it != range.second;) {
      if (it->second.owner == handler.owner) {
        connections_by_id_.erase(it->second.id);
        if (mark_queued_for_disconnection) {
          it->second.is_queued_for_disconnection = true;
          ++it;
        } else {
          it = map->erase(it);
        }
      } else {
        ++it;
      }
    }
  }
  return map->empty();
}

void DispatcherEventHandlerMap::DisconnectImpl(
    NodeHandle node, HashValue type, TaggedEventHandler&& handler,
    bool mark_queued_for_disconnection) {
  assert(!handler);
  assert(handler.id != kNullConnectionId || handler.owner.IsValid());

  ++disconnect_count_;

  // If DisconnectAll, check all nodes.
  if (type == 0 && handler.id == kNullConnectionId) {
    auto node_range = std::make_pair(map_.begin(), map_.end());
    std::vector<NodeHandle> empty_nodes;

    for (auto node_iter = node_range.first; node_iter != node_range.second;
         ++node_iter) {
      if (DisconnectByType(&node_iter.value(), type, handler,
                           mark_queued_for_disconnection)) {
        empty_nodes.push_back(node_iter->first);
      }
    }
    for (auto& empty_node : empty_nodes) {
      map_.erase(empty_node);
    }
  } else {
    auto iter = map_.find(node);
    if (iter != map_.end() && DisconnectByType(&iter.value(), type, handler,
                                               mark_queued_for_disconnection)) {
      map_.erase(node);
    }
  }

  --disconnect_count_;

  if (disconnect_count_ == 0) {
    DrainCommandQueue(disconnect_command_queue_);
  }
}

void DispatcherEventHandlerMap::DrainCommandQueue(CommandQueue& command_queue) {
  if (command_queue.empty()) {
    return;
  }

  CommandQueue swapped_queue;
  std::swap(swapped_queue, command_queue);
  for (auto& cmd : swapped_queue) {
    NodeHandle& cmd_node = std::get<0>(cmd);
    HashValue& cmd_type = std::get<1>(cmd);
    TaggedEventHandler& cmd_handler = std::get<2>(cmd);
    // A non-null EventHandler implies that the operation is an add.
    if (cmd_handler) {
      ConnectImpl(cmd_node, cmd_type, std::move(cmd_handler));
    } else {
      DisconnectImpl(cmd_node, cmd_type, std::move(cmd_handler));
    }
  }
}

NodeHandle DispatcherEventHandlerMap::Dispatch(NodeHandle node, HashValue type,
                                               Event& event) {
  // If sent to a dead node, discard the event.
  if (!node.IsDefaultValue() && !node.IsValid()) {
    return node;
  }

  const Event::PropagationMode mode = event.GetPropagationMode();
  event.SetOriginatingNode(node);
  while (true) {
    event.SetTargetNode(node);
    PropagationResult result = DispatchImpl(node, type, event);

    // Only dispatch once if kNone, at global scope, or kAccepted.
    if (mode == Event::kNone || !node || result != Dispatcher::kContinue) {
      break;
    }
    // Keep propagating.
    node = node->GetParent();
  }

  event.SetConnectionInfo(nullptr, 0);

  return node;
}

Dispatcher::PropagationResult DispatcherEventHandlerMap::DispatchImpl(
    NodeHandle node, HashValue type, Event& event) {
  PropagationResult result = Dispatcher::kContinue;
  // NOTE: if you crash in this function, it may be because you destroyed an
  // an Entity from inside an event handler.
  // TODO: Implement View::QueueForDestruction or make default.
  ++dispatch_count_;
  auto iter = map_.find(node);
  if (iter != map_.end()) {
    auto range = iter->second.equal_range(type);
    for (auto it = range.first; it != range.second; ++it) {
      const TaggedEventHandler& tagged_event_handler = it->second;
      event.SetConnectionInfo(this, tagged_event_handler.id);
      if (tagged_event_handler(event) == Dispatcher::kAccept) {
        result = Dispatcher::kAccept;
      }
    }
  }
  --dispatch_count_;

  if (dispatch_count_ == 0) {
    DrainCommandQueue(dispatch_command_queue_);
  }
  return result;
}

size_t DispatcherEventHandlerMap::Size() const {
  size_t sum = 0;
  for (const auto& map : map_) {
    sum += map.second.size();
  }
  return sum;
}

size_t DispatcherEventHandlerMap::GetHandlerCount(NodeHandle node,
                                                  HashValue type) const {
  auto iter = map_.find(node);
  if (iter != map_.end()) {
    return iter->second.count(type);
  }
  return 0;
}

void DispatcherEventHandlerMap::SetForgetter(NodeHandle node, HashValue type,
                                             ConnectionId id,
                                             Invocable<void()> forgetter) {
  if (!id) {
    return;
  }
  auto iter = map_.find(node);
  if (iter == map_.end()) {
    return;
  }
  auto& map = iter.value();

  auto range = map.equal_range(type);
  for (auto it = range.first; it != range.second; ++it) {
    if (it->second.id == id) {
      it->second.forgetters.emplace_back(std::move(forgetter));
      return;
    }
  }
}
}  // namespace imp
