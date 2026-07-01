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

#include <cstddef>
#include <memory>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/inlined_vector.h"
#include "absl/log/check.h"
#include "absl/types/optional.h"
#include "core/common/hash.h"
#include "core/common/invocable.h"
#include "core/ncsb/dispatcher/connection_holder.h"
#include "core/ncsb/dispatcher/connection_id.h"
#include "core/ncsb/dispatcher/connection_owner.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"

namespace imp {

// Value used to represent all types when disconnecting.
constexpr HashValue kAllTypes = 0;

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
    TaggedEventHandler(TaggedEventHandler&& other) noexcept
        : id(other.id),
          owner(std::move(other.owner)),
          fn(std::move(other.fn)),
          forgetters(std::move(other.forgetters)),
          is_queued_for_disconnection(other.is_queued_for_disconnection) {
      other.forgetters.clear();
      other.fn = absl::nullopt;
    }

    TaggedEventHandler& operator=(const TaggedEventHandler&) = delete;
    TaggedEventHandler& operator=(TaggedEventHandler&& other) noexcept {
      if (this != &other) {
        // If assigned over, this state is destroyed. Call its forgetters.
        for (auto& forgetter : forgetters) {
          forgetter();
        }
        id = other.id;
        owner = std::move(other.owner);
        fn = std::move(other.fn);
        forgetters = std::move(other.forgetters);
        is_queued_for_disconnection = other.is_queued_for_disconnection;
        other.forgetters.clear();
        other.fn = absl::nullopt;
      }
      return *this;
    }

    PropagationResult operator()(const Event& event) const {
      

      if (is_queued_for_disconnection) {
        return PropagationResult::kContinue;
      }

      if (auto* prop_fn = std::get_if<EventHandlerPropagationResult>(&*fn)) {
        return (*prop_fn)(event);
      } else {
        std::get<EventHandlerVoid> (*fn)(event);
        return PropagationResult::kContinue;
      }
    }

    explicit operator bool() const noexcept {
      if (!fn || is_queued_for_disconnection) {
        return false;
      }

      if (auto* prop_fn = std::get_if<EventHandlerPropagationResult>(&*fn)) {
        return static_cast<bool>(*prop_fn);
      } else {
        return static_cast<bool>(std::get<EventHandlerVoid>(*fn));
      }
    }

    ConnectionId id;
    ConnectionOwner owner;
    absl::optional<EventHandlerVariant> fn;
    // Inline two elements to avoid heap allocation for the common case.
    //
    // Example:
    //   node->Connect([](const MyEvent& e) { ... }, this);
    //
    // There will be one forgetter for when node is destroyed and another for
    // when the owner (this) is destroyed.
    //
    // One forgetter is possible when connecting to the global scope, or zero if
    // not using an owner.
    absl::InlinedVector<Invocable<void()>, 2> forgetters;
    bool is_queued_for_disconnection = false;
  };

  struct ConnectCommand {
    NodeHandle node;
    HashValue type;
    TaggedEventHandler handler;
  };

  struct DisconnectCommand {
    NodeHandle node;
    HashValue type;
    ConnectionId id;
    ConnectionOwner owner;
  };

  using Command = std::variant<ConnectCommand, DisconnectCommand>;

  // Deferred queue of add/remove commands while commands are in progress..
  using CommandQueue = std::vector<Command>;

  using HandlerList = absl::InlinedVector<TaggedEventHandler, 1>;

  using NodeToHandlersMap = absl::flat_hash_map<NodeHandle, HandlerList>;

  using TypeToNodeToHandlersMap =
      absl::flat_hash_map<HashValue, NodeToHandlersMap>;

  // Actually connects the EventHandler.
  void ConnectImpl(NodeHandle node, HashValue type,
                   TaggedEventHandler&& handler);

  // Disconnects the EventHandler.
  //
  // If |mark_queued_for_disconnection| is true, the handler will be marked as
  // queued for disconnection so that it will not be invoked, but it will not
  // actually be disconnected until the current operation is complete.
  //
  // The overloads to this function are used to handle the different cases of
  // how a connection can be disconnected (i.e. by node, by type, by id, by
  // owner, etc.)

  void DisconnectImpl(NodeHandle node, HashValue type, ConnectionId id,
                      const ConnectionOwner& owner,
                      bool mark_queued_for_disconnection);

  void DisconnectByIdImpl(NodeHandle node, HashValue type, ConnectionId id,
                          bool mark_queued_for_disconnection);

  void DisconnectByOwnerImpl(NodeHandle node, HashValue type,
                             const ConnectionOwner& owner,
                             bool mark_queued_for_disconnection);

  void DisconnectAllByOwnerImpl(const ConnectionOwner& owner,
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
  TypeToNodeToHandlersMap map_;

  // Map to store additional data by id to make disconnecting easier.
  absl::flat_hash_map<ConnectionId, std::pair<NodeHandle, HashValue>>
      connections_by_id_;
};

Dispatcher::Dispatcher() { handlers_ = std::make_shared<EventHandlerMap>(); }

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
  
  return ConnectImpl(node, type, std::move(handler), ConnectionOwner());
}

void Dispatcher::Disconnect(NodeHandle node, HashValue type, ConnectionId id) {
  
  handlers_->Disconnect(node, type, id, ConnectionOwner());
}

void Dispatcher::Disconnect(ConnectionId id) {
  
  handlers_->Disconnect(id);
}

void Dispatcher::DisconnectAll(const ConnectionOwner& owner) {
  if (!owner.IsValid()) {
    return;
  }

  // Calling Remove() should DCHECK type != 0 or id != kNullConnectionId
  // unless setting them manually.
  handlers_->Disconnect(NodeHandle(), 0, kNullConnectionId, owner);
}

NodeHandle Dispatcher::SendImpl(NodeHandle node, HashValue type_hash,
                                const Event& event) {
  // Event can't be const so that it's possible for the dispatcher to set
  // the originating node, target node, connection info, etc.
  // TODO: Eliminate this const_cast.
  Event& non_const_event = const_cast<Event&>(event);
  return handlers_->Dispatch(node, type_hash, non_const_event);
}

Dispatcher::Connection Dispatcher::ConnectImpl(NodeHandle node, HashValue type,
                                               EventHandlerVariant handler,
                                               const ConnectionOwner& owner) {
  // All callers for ConnectImpl() DCHECK type != 0.
  const ConnectionId id = ++id_;
  handlers_->Connect(node, type, id, owner, std::move(handler));
  auto connection = Connection(handlers_, node, type, id);

  // If we are targeting a node and not being owned by that node, also cleanup
  // the connection when the target node is destroyed.
  if (!node.IsDefaultValue() && node != owner.GetNode()) {
    KeepAlive(node, connection);
  }
  return connection;
}

void Dispatcher::DisconnectImpl(NodeHandle node, HashValue type,
                                const ConnectionOwner& owner) {
  
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
    dispatch_command_queue_.push_back(
        ConnectCommand{node, type, std::move(handler)});
  } else if (disconnect_count_ > 0) {
    disconnect_command_queue_.push_back(
        ConnectCommand{node, type, std::move(handler)});
  } else {
    ConnectImpl(node, type, std::move(handler));
  }
}

void DispatcherEventHandlerMap::Disconnect(NodeHandle node, HashValue type,
                                           ConnectionId id,
                                           const ConnectionOwner& owner) {
  if (dispatch_count_ > 0) {
    DisconnectImpl(node, type, id, owner, true);
    dispatch_command_queue_.push_back(DisconnectCommand{node, type, id, owner});
  } else if (disconnect_count_ > 0) {
    disconnect_command_queue_.push_back(
        DisconnectCommand{node, type, id, owner});
  } else {
    DisconnectImpl(node, type, id, owner, false);
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
  
  
  connections_by_id_[handler.id] = {node, type};
  map_[type][node].emplace_back(std::move(handler));
}

void DispatcherEventHandlerMap::DisconnectImpl(
    NodeHandle node, HashValue type, ConnectionId id,
    const ConnectionOwner& owner, bool mark_queued_for_disconnection) {
  // Used to handle reentrant calls to the dispatcher.
  //
  // Allows us to track state changes and resolve them after the reentrant
  // call returns.
  ++disconnect_count_;

  if (id != kNullConnectionId) {
    // The id is specified, so we can disconnect by id.
    DisconnectByIdImpl(node, type, id, mark_queued_for_disconnection);
  } else if (type == kAllTypes) {
    // The type is specified as kAllTypes, so we can disconnect all events for
    // all types on the owner.
    DisconnectAllByOwnerImpl(owner, mark_queued_for_disconnection);
  } else {
    // Otherwise, we disconnect by owner for the given type of event.
    DisconnectByOwnerImpl(node, type, owner, mark_queued_for_disconnection);
  }

  --disconnect_count_;

  if (disconnect_count_ == 0) {
    DrainCommandQueue(disconnect_command_queue_);
  }
}

void DispatcherEventHandlerMap::DisconnectByIdImpl(
    NodeHandle node, HashValue type, ConnectionId id,
    bool mark_queued_for_disconnection) {
  
  

  // Lookup the type of event.
  auto type_iter = map_.find(type);
  if (type_iter == map_.end()) {
    return;
  }

  // Lookup the handlers for the node within the type.
  // Global scope is represented by a default node.
  NodeToHandlersMap& node_map = type_iter->second;
  auto node_iter = node_map.find(node);
  if (node_iter == node_map.end()) {
    return;
  }

  // Iterate over the handlers and disconnect the handler with the given id.
  HandlerList& handlers = node_iter->second;
  for (auto it = handlers.begin(); it != handlers.end(); ++it) {
    if (it->id == id) {
      connections_by_id_.erase(id);
      if (mark_queued_for_disconnection) {
        it->is_queued_for_disconnection = true;
      } else {
        handlers.erase(it);
      }
      break;
    }
  }

  // Remove entries if they've become empty.
  if (handlers.empty() && !mark_queued_for_disconnection) {
    node_map.erase(node_iter);
    if (node_map.empty()) {
      map_.erase(type_iter);
    }
  }
}

void DispatcherEventHandlerMap::DisconnectByOwnerImpl(
    NodeHandle node, HashValue type, const ConnectionOwner& owner,
    bool mark_queued_for_disconnection) {
  
  

  // Lookup the type of event.
  auto type_iter = map_.find(type);
  if (type_iter == map_.end()) {
    return;
  }

  // Lookup the handlers for the node within the type.
  // Global scope is represented by a default node.
  NodeToHandlersMap& node_map = type_iter->second;
  auto node_iter = node_map.find(node);
  if (node_iter == node_map.end()) {
    return;
  }

  // Iterate over the handlers and disconnect the handlers with the given owner.
  HandlerList& handlers = node_iter->second;
  for (auto it = handlers.begin(); it != handlers.end();) {
    if (it->owner == owner) {
      connections_by_id_.erase(it->id);
      if (mark_queued_for_disconnection) {
        it->is_queued_for_disconnection = true;
        ++it;
      } else {
        it = handlers.erase(it);
      }
    } else {
      ++it;
    }
  }

  // Remove entries if they've become empty.
  if (handlers.empty() && !mark_queued_for_disconnection) {
    node_map.erase(node_iter);
    if (node_map.empty()) {
      map_.erase(type_iter);
    }
  }
}

void DispatcherEventHandlerMap::DisconnectAllByOwnerImpl(
    const ConnectionOwner& owner, bool mark_queued_for_disconnection) {
  

  // Iterate over all types of events.
  for (auto type_iter = map_.begin(); type_iter != map_.end();) {
    NodeToHandlersMap& node_map = type_iter->second;

    // Iterate over all nodes for the given event type.
    for (auto node_iter = node_map.begin(); node_iter != node_map.end();) {
      HandlerList& handlers = node_iter->second;

      // Iterate over the handlers and disconnect the handlers with the given
      // owner.
      for (auto it = handlers.begin(); it != handlers.end();) {
        if (it->owner == owner) {
          connections_by_id_.erase(it->id);
          if (mark_queued_for_disconnection) {
            it->is_queued_for_disconnection = true;
            ++it;
          } else {
            it = handlers.erase(it);
          }
        } else {
          ++it;
        }
      }

      // Remove entries if they've become empty.
      if (handlers.empty() && !mark_queued_for_disconnection) {
        node_map.erase(node_iter++);
      } else {
        ++node_iter;
      }
    }

    // Remove entries if they've become empty.
    if (node_map.empty() && !mark_queued_for_disconnection) {
      map_.erase(type_iter++);
    } else {
      ++type_iter;
    }
  }
}

void DispatcherEventHandlerMap::DrainCommandQueue(CommandQueue& command_queue) {
  if (command_queue.empty()) {
    return;
  }

  CommandQueue swapped_queue;
  std::swap(swapped_queue, command_queue);
  for (auto& cmd : swapped_queue) {
    if (auto* connect = std::get_if<ConnectCommand>(&cmd)) {
      ConnectImpl(connect->node, connect->type, std::move(connect->handler));
    } else if (auto* disconnect = std::get_if<DisconnectCommand>(&cmd)) {
      DisconnectImpl(disconnect->node, disconnect->type, disconnect->id,
                     disconnect->owner, false);
    }
  }
}

NodeHandle DispatcherEventHandlerMap::Dispatch(NodeHandle node, HashValue type,
                                               Event& event) {
  // If sent to a destroyed node, discard the event.
  //
  // A default node represents the global scope of the dispatcher.
  if (!node.IsDefaultValue() && !node.IsValid()) {
    return node;
  }

  const Event::PropagationMode mode = event.GetPropagationMode();

  // Set initial event properties.
  event.SetOriginatingNode(node);
  event.SetTargetNode(node);

  auto type_iter = map_.find(type);
  if (type_iter == map_.end()) {
    // No handlers for this type of event.
    return node;
  }

  NodeToHandlersMap& node_map = type_iter->second;

  // Used to handle reentrant calls to the dispatcher.
  //
  // Allows us to track state changes and resolve them after the reentrant
  // call returns.
  ++dispatch_count_;

  while (true) {
    PropagationResult result = Dispatcher::kContinue;

    auto node_iter = node_map.find(node);
    if (node_iter != node_map.end()) {
      for (const TaggedEventHandler& tagged_event_handler : node_iter->second) {
        event.SetConnectionInfo(this, tagged_event_handler.id);
        if (tagged_event_handler(event) == Dispatcher::kAccept) {
          result = Dispatcher::kAccept;
        }
      }
    }

    // Only dispatch once if kNone, at global scope, or kAccepted.
    if (mode == Event::kNone || !node || result != Dispatcher::kContinue) {
      break;
    }

    // Keep propagating the event up all the ancestors in the hierarchy.
    node = node->GetParent();
    event.SetTargetNode(node);
  }

  --dispatch_count_;

  if (dispatch_count_ == 0) {
    DrainCommandQueue(dispatch_command_queue_);
  }

  event.SetConnectionInfo(nullptr, 0);

  return node;
}

size_t DispatcherEventHandlerMap::Size() const {
  size_t sum = 0;
  for (const auto& [type, node_map] : map_) {
    for (const auto& [node, vec] : node_map) {
      sum += vec.size();
    }
  }
  return sum;
}

size_t DispatcherEventHandlerMap::GetHandlerCount(NodeHandle node,
                                                  HashValue type) const {
  auto type_iter = map_.find(type);
  if (type_iter != map_.end()) {
    auto node_iter = type_iter->second.find(node);
    if (node_iter != type_iter->second.end()) {
      return node_iter->second.size();
    }
  }
  return 0;
}

void DispatcherEventHandlerMap::SetForgetter(NodeHandle node, HashValue type,
                                             ConnectionId id,
                                             Invocable<void()> forgetter) {
  if (!id) {
    return;
  }

  auto type_iter = map_.find(type);
  if (type_iter != map_.end()) {
    auto node_iter = type_iter->second.find(node);
    if (node_iter != type_iter->second.end()) {
      for (auto& handler : node_iter->second) {
        if (handler.id == id) {
          handler.forgetters.emplace_back(std::move(forgetter));
          return;
        }
      }
    }
  }
}
}  // namespace imp
