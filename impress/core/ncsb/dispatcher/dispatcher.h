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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_DISPATCHER_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_DISPATCHER_H_

#include <assert.h>
#include <stdint.h>

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

#include "absl/types/variant.h"
#include "core/common/hash.h"
#include "core/common/holdable.h"
#include "core/common/invocable.h"
#include "core/common/rememberer.h"
#include "core/common/trace.h"
#include "core/ncsb/dispatcher/connection_id.h"
#include "core/ncsb/dispatcher/connection_owner.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/dispatcher/event_type_helper.h"
#include "core/ncsb/node_handle.h"
#include "core/view/utils/macros.h"

namespace imp {

template <typename Owner>
using EnableIfConnectionOwner =
    std::enable_if_t<std::is_constructible<ConnectionOwner, Owner>::value, int>;

class DispatcherEventHandlerMap;
typedef std::shared_ptr<DispatcherEventHandlerMap> DispatcherEventHandlerMapPtr;
typedef std::weak_ptr<DispatcherEventHandlerMap>
    DispatcherEventHandlerMapWeakPtr;

// Handle for event registrations with the Dispatcher. This handle is used to
// disconnect the connection.
// Note: this is declared outside of the Dispatcher class because we need to
// use it in NodeDeclaration as a forward-declared type which doesn't support
// nested classes.
class DispatcherConnection {
 public:
  DispatcherConnection();
  DispatcherConnection(const DispatcherEventHandlerMapPtr& handlers,
                       NodeHandle node, HashValue type, ConnectionId id);

  // Disconnect event handler from the dispatcher.  It is safe to call this
  // function multiple times.
  void Disconnect();

  // Get the ConnectionId that can be passed to Dispatcher::Disconnect()
  ConnectionId GetId() const;
  HashValue GetTypeHash() const;
  NodeHandle GetNode() const;

 private:
  NodeHandle node_;
  HashValue type_;
  ConnectionId id_;
  DispatcherEventHandlerMapWeakPtr handlers_;
};

// A simple event handling mechanism.
//
// The easiest way to explain it is probably through code:
//
// struct SomeEvent : public imp::Event {
//   int x = 0;
// };
//
// struct SomeClass {
//   void HandleEvent(const SomeEvent& e) {
//     // Do something with |e|.
//   }
// };
//
// void GlobalHandleEvent(const SomeEvent& e) {
//   // Do something with |e|.
// }
//
// void main() {
//   Dispatcher dispatcher;
//   auto c1 = dispatcher.Connect([](const SomeEvent& event) {
//     GlobalHandleEvent(event);
//   });
//
//   SomeClass cls;
//   auto c2 = dispatcher.Connect([&](const SomeEvent& event) {
//     cls.HandleEvent(event);
//   });
//
//   dispatcher.Send(SomeEvent{123});
// }
//
// Running the above will result in calls to GlobalHandleEvent and
// cls.HandleEvent with SomeEvent.x == 123.
//
// The Dispatcher uses a "double-dispatch" mechanism to handle events that are
// "sent" through it.  Internally, the Dispatcher stores EventHandler functor
// that first takes the base type Event and converts it back to the Event
// subclass instance and then calls the functor connected with the Event.  (The
// use of two functors is where the term "double-dispatch" gets its name.)
// Events must inherit from imp::Event.
//
// The Dispatcher stores a map of EventHandlers associated with the TypeHashes
// of the Event subclasses. When an Event is "sent" through the dispatcher, the
// list of EventHandlers associated with the type hash of the sent event is
// looked up, and the event is passed to those handlers.
//
// The Connect() function returns a ScopedConnection object which must be
// stored by the client.  When this object goes out of scope, the connected
// function will be removed from the Dispatcher.
//
// The Connect() function also includes overloads that allow you to pass in an
// Owner to control the lifetime of the connection.  The Owner must be
// convertible to ConnectionOwner.
//
// If the Owner implements the Remember protocol (see
// //third_party/impress/core/common/rememberer.h for details),
// the connection will automatically be disconnected when the Owner 'forgets'
// it's remembered set (typically on destruction).  Common Owners like
// View, Component, and Node all implement the Rememberer protocol.
//
// If the Owner is a raw pointer, then the connection needs to
// be manually disconnected.
//
// If Connect is called with an Owner*, then a non-scoped Connection
// object is returned. The client can then call Disconnect on the Connection
// object, use Dispatcher's Disconnect with the object's ConnectionId, or
// disconnect from the Dispatcher using the same owner as a way to
// identify the connection to close.  A single owner pointer can be
// associated with multiple connections.
//
// In addition to sending/receiving concrete Event subclasses, clients can
// connect to the base Event type with a type hash representing the subclass.
// This allows clients to process events in a more generic way. This is useful
// for cases where you want to determine at runtime what type of event to
// connect to, for example when listening to events across language boundaries.
// Example:
// google3/third_party/impress/core/scripting/scripting_system.cc
//
// Connections are all made to a NodeHandle, which will only receive Events sent
// to that NodeHandle.  Calling these API without the NodeHandle or passing in
// the default one corresponds to global scope.  Connections targeted to a
// non-null NodeHandle are also cleaned up when the target is destroyed, even if
// the owner is different.
//
// Events have a PropagationMode that controls how they will propagate. The
// default kBubbleUp will go up the Node hierarchy up to and including the
// global scope. EventHandlers can return Dispatcher::kAccept to stop
// propagation at their Node. Having no connection, returning nothing, or
// returning Dispatcher::kContinue will let propagation continue. Events will
// have their Originating and Target nodes updated for each Handler (even though
// they are publicly const).
class Dispatcher {
 private:
  // Important: All queries for the HashValue of Events in Dispatcher should use
  // EventTypeHelper::GetEventTypeHash<T>() instead of
  // type_traits::kTypeHash<T>.

  // Internal class that stores the map of HashValue to EventHandlers (and
  // associated typedefs).
  using EventHandlerMap = DispatcherEventHandlerMap;
  using EventHandlerMapPtr = DispatcherEventHandlerMapPtr;
  using EventHandlerMapWeakPtr = DispatcherEventHandlerMapWeakPtr;

  friend class DispatcherConnection;

 public:
  // Returned from EventHandler to control propagation.
  enum PropagationResult {
    // Propagation continues as normal, going up parents up to and including the
    // global scope. This is the default if nothing is returned.
    kContinue,
    // Propagation stops at this node.  All other Connections on this Node
    // are still invoked.
    kAccept,
  };

  // The underlying functor used for handling events.
  using EventHandlerPropagationResult =
      Invocable<PropagationResult(const Event&)>;
  using EventHandlerVoid = Invocable<void(const Event&)>;
  using EventHandlerVariant =
      absl::variant<EventHandlerPropagationResult, EventHandlerVoid>;

  // Connection object returned by Dispatcher::Connect which must be explicitly
  // disconnected by calling Connection::Disconnect().
  using Connection = DispatcherConnection;

  // ScopedConnection object returned by Dispatcher::Connect which will
  // automatically disconnect the connection when this object goes out of
  // scope.
  class IMP_WARN_UNUSED_RESULT ScopedConnection {
   public:
    ScopedConnection() {}
    ScopedConnection(Connection c);  // NOLINT(runtime/explicit)
    ScopedConnection(ScopedConnection&& rhs);
    ScopedConnection& operator=(ScopedConnection&& rhs);
    ~ScopedConnection();

    Connection GetConnection() const { return connection_; }

    // Explicitly disconnect the connection rather than waiting for the
    // ScopedConnection to go out of scope.
    void Disconnect();

   private:
    Connection connection_;

    ScopedConnection(const ScopedConnection&) = delete;
    ScopedConnection& operator=(const ScopedConnection&) = delete;
  };

  Dispatcher();
  virtual ~Dispatcher() {}

  // Sends an event to all functions Connected with the dispatcher to |node|.
  // The |EventType| object must inherit from imp::Event. The PropagationMode
  // can be set to control propagation. Calling without the NodeHandle or
  // passing in the default one sends the event to global scope.
  //
  // Note: The base Event properties (TargetNode, OriginatingNode) will still be
  // modified even though |event| is const.
  //
  // Note: Sending to a destroyed node will ignored.
  //
  // Returns: The last NodeHandle that the |event| was sent to, or the null
  // NodeHandle for global scope.
  template <typename EventType>
  NodeHandle Send(const EventType& event);
  template <typename EventType>
  NodeHandle Send(NodeHandle node, const EventType& event);

  // Connects the |handler| to listen to events sent to |node|, where the type
  // of event is specified by the signature of the |handler| (eg. void(const
  // EventType&)). The handler can return a Dispatcher::PropagationResult or
  // void, which defaults to Dispatcher::kContinue. Calling without the
  // NodeHandle or passing in the default one connects to global scope.
  //
  // Returns: ScopedConnection which will automatically disconnect the function
  //   when it goes out of scope.
  template <typename Fn>
  ScopedConnection Connect(Fn&& handler);
  template <typename Fn>
  ScopedConnection Connect(NodeHandle node, Fn&& handler);

  // Connects |handler| to base class Event instances of the specified |type|
  // sent to |node|.
  //
  // This is useful for cases where you want to determine at runtime what type
  // of event to connect to, for example when listening to events across
  // language boundaries. Example:
  // google3/third_party/impress/core/scripting/scripting_system.cc
  //
  // The handler can return a Dispatcher::PropagationResult or void, which
  // defaults to Dispatcher::kContinue. Calling without the NodeHandle or
  // passing in the default one connects to global scope.
  //
  // Returns: ScopedConnection which will automatically disconnect the function
  //   when it goes out of scope.
  ScopedConnection Connect(HashValue type,
                           EventHandlerPropagationResult handler);
  ScopedConnection Connect(HashValue type, EventHandlerVoid handler);

  ScopedConnection Connect(NodeHandle node, HashValue type,
                           EventHandlerPropagationResult handler);
  ScopedConnection Connect(NodeHandle node, HashValue type,
                           EventHandlerVoid handler);

  // Connects the |handler| to listen to events sent to |node|, where the type
  // of event is specified by the signature of the |handler| (ie. void(const
  // EventType&)). The handler can return a Dispatcher::PropagationResult or
  // void, which defaults to Dispatcher::kContinue. Calling without the
  // NodeHandle or passing in the default one connects to global scope.
  //
  // An |owner| is used to automatically disconnect the function based on the
  // lifecycle of the |owner|. It must be a pointer or a pointer-like object.
  //
  // If |owner| is a node, then the function is disconnected when the node is
  // destroyed.
  // If |owner| is a component, then the function is disconnected
  // when the component is removed.
  // If |owner| is a pointer to a view, then the function is disconnected when
  // the view is cleaned up.
  // If |owner| is any other type, it must implement a Remember method. See
  // google3/third_party/impress/core/common/rememberer.h for more
  // details.
  //
  // Returns: Connection which can be used to disconnect the function directly.
  template <typename Owner, typename Fn, EnableIfConnectionOwner<Owner> = 0>
  Connection Connect(Fn&& handler, Owner owner);
  template <typename Owner, typename Fn, EnableIfConnectionOwner<Owner> = 0>
  Connection Connect(NodeHandle node, Fn&& handler, Owner owner);

  // Connects |handler| to base class Event instances of the specified |type|
  // sent to |node|.
  //
  // This is useful for cases where you want to determine at runtime what type
  // of event to connect to, for example when listening to events across
  // language boundaries. Example:
  // google3/third_party/impress/core/scripting/scripting_system.cc
  //
  // The handler can return a Dispatcher::PropagationResult or void. which
  // defaults to Dispatcher::kContinue. Calling without the NodeHandle or
  // passing in the default one connects to global scope.
  //
  // An |owner| is used to automatically disconnect the function based on the
  // lifecycle of the |owner|. It must be a pointer or a pointer-like object.
  //
  // If |owner| is a node, then the function is disconnected when the node is
  // destroyed.
  // If |owner| is a component, then the function is disconnected
  // when the component is removed.
  // If |owner| is a pointer to a view, then the function is disconnected when
  // the view is cleaned up.
  // If |owner| is any other type, it must implement a Remember method. See
  // google3/third_party/impress/core/common/rememberer.h for more
  // details.
  //
  // Returns: Connection which can be used to disconnect the function.
  template <typename Owner, EnableIfConnectionOwner<Owner> = 0>
  Connection Connect(HashValue type, EventHandlerPropagationResult handler,
                     Owner&& owner);
  template <typename Owner, EnableIfConnectionOwner<Owner> = 0>
  Connection Connect(HashValue type, EventHandlerVoid handler, Owner&& owner);
  template <typename Owner, EnableIfConnectionOwner<Owner> = 0>
  Connection Connect(NodeHandle node, HashValue type,
                     EventHandlerPropagationResult handler, Owner&& owner);
  template <typename Owner, EnableIfConnectionOwner<Owner> = 0>
  Connection Connect(NodeHandle node, HashValue type, EventHandlerVoid handler,
                     Owner&& owner);

  // Disconnect the connection with specified |node|, |type| and |id|.
  void Disconnect(NodeHandle node, HashValue type, ConnectionId id);

  // Disconnect the connection with specified |id|.
  void Disconnect(ConnectionId id);

  // Disconnects all functions listening to the |EventType| to |node| associated
  // with the specified |owner|. Calling without the NodeHandle or passing in
  // the default one disconnects from the global scope.
  template <
      typename EventType, typename Owner,
      std::enable_if_t<std::is_constructible<ConnectionOwner, Owner>::value &&
                           std::is_base_of<Event, EventType>::value,
                       int> = 0>
  void Disconnect(Owner&& owner);
  template <
      typename EventType, typename Owner,
      std::enable_if_t<std::is_constructible<ConnectionOwner, Owner>::value &&
                           std::is_base_of<Event, EventType>::value,
                       int> = 0>
  void Disconnect(NodeHandle node, Owner&& owner);

  // Disconnects all functions listening to events of the specified |type| to
  // |node| associated with the specified |owner|. Calling without the
  // NodeHandle or passing in the default one disconnects from the global scope.
  template <typename Owner,
            std::enable_if_t<
                std::is_constructible<ConnectionOwner, Owner>::value, int> = 0>
  void Disconnect(HashValue type, Owner&& owner);
  template <typename Owner,
            std::enable_if_t<
                std::is_constructible<ConnectionOwner, Owner>::value, int> = 0>
  void Disconnect(NodeHandle node, HashValue type, Owner&& owner);

  // Disconnects all functions with the specified |owner|.
  template <typename Owner,
            std::enable_if_t<
                std::is_constructible<ConnectionOwner, Owner>::value, int> = 0>
  void DisconnectAll(Owner owner);

  // Returns the number of functions currently registered with this dispatcher.
  size_t GetHandlerCount() const;

  // Returns the number of functions listening for an event of |type| on |node|.
  // Calling without the NodeHandle or passing in the default one returns for
  // the global scope.
  size_t GetHandlerCount(HashValue type) const;
  size_t GetHandlerCount(NodeHandle node, HashValue type) const;

 private:
  ScopedConnection ConnectVariant(HashValue type, EventHandlerVariant handler);
  ScopedConnection ConnectVariant(NodeHandle node, HashValue type,
                                  EventHandlerVariant handler);

  template <typename Owner, EnableIfConnectionOwner<Owner> = 0>
  Connection ConnectVariant(HashValue type, EventHandlerVariant handler,
                            Owner&& owner);
  template <typename Owner, EnableIfConnectionOwner<Owner> = 0>
  Connection ConnectVariant(NodeHandle node, HashValue type,
                            EventHandlerVariant handler, Owner&& owner);

  // Helper function declaration that is used to extract the Event type from
  // an event handler.
  template <typename Fn, typename Arg, typename Ret>
  static Arg ConnectHelper(Ret (Fn::*)(const Arg&) const);

  // Mutable helper function declaration that is used to extract the Event type
  // from an event handler.
  template <typename Fn, typename Arg, typename Ret>
  static Arg ConnectHelper(Ret (Fn::*)(const Arg&));

  // Helper function for properly wrapping a function in a type-erased
  // EventHandlerVariant.
  template <typename Fn>
  static EventHandlerVariant EventHandlerVariantFromFn(Fn&& fn);

  // Passes the Event to all the functions Connected with the Dispatcher
  // with the same HashValue as the type_hash. Returns the last node that was
  // sent to, or the null NodeHandle for global scope.
  NodeHandle SendImpl(NodeHandle node, HashValue type_hash, const Event& event);

  // Creates the actual Handler instance, registers it with the map, and
  // returns the corresponding Connection object.
  Connection ConnectImpl(NodeHandle node, HashValue type,
                         EventHandlerVariant handler,
                         const ConnectionOwner& owner);

  // Removes the Handler that matches the |type| and |owner|.
  void DisconnectImpl(NodeHandle node, HashValue type,
                      const ConnectionOwner& owner);

  void DisconnectAll(const ConnectionOwner& owner);

  // Helps with KeepAlive/DisconnectAll behavior.
  // Owner must be convertible to a ConnectionOwner.
  // This variant is used if Owner has a 'Remember' method.
  template <typename Owner>
  void KeepAlive(Owner owner, const Connection& connection);

  void SetForgetter(const Connection& conn, Invocable<void()> forgetter);

  // Autoincrementing value for generating unique connection IDs.
  ConnectionId id_ = 0;
  // kNullConnectionId represents "all connections" and kNullType represents
  // "all types".
  // These are used for ConnectToAll(), DisconnectAll(), and DisconnectByType().

  // Map of HashValue to EventHandlers.  Uses a shared_ptr to allow Connection
  // objects to safely "disconnect" from Dispatchers that have been destroyed.
  EventHandlerMapPtr handlers_;

  Dispatcher(const Dispatcher&) = delete;
  Dispatcher& operator=(const Dispatcher&) = delete;
};

template <typename EventType>
inline NodeHandle Dispatcher::Send(const EventType& event) {
  return Send(NodeHandle(), event);
}

template <typename EventType>
inline NodeHandle Dispatcher::Send(NodeHandle node, const EventType& event) {
  IMP_TRACE_TEMPLATED(EventType);
  return SendImpl(node, EventTypeHelper::GetEventTypeHash<EventType>(), event);
}

template <typename Fn>
Dispatcher::ScopedConnection Dispatcher::Connect(Fn&& handler) {
  return Connect(NodeHandle(), std::forward<Fn>(handler));
}

template <typename Fn>
Dispatcher::ScopedConnection Dispatcher::Connect(NodeHandle node,
                                                 Fn&& handler) {
  using FnType = typename std::remove_reference<Fn>::type;
  using EventType = decltype(ConnectHelper(&FnType::operator()));
  const HashValue type = EventTypeHelper::GetEventTypeHash<EventType>();

  return ConnectVariant(node, type,
                        EventHandlerVariantFromFn(std::forward<Fn>(handler)));
}

template <typename Owner>
void Dispatcher::KeepAlive(Owner owner, const Connection& connection) {
  static_assert(CanTypeRemember<Owner>::value,
                "Owner must contain a Remember method by either inheriting "
                "from imp::Rememberer or defining a Remember method itself.");

  if (owner) {
    SetForgetter(connection,
                 owner->Remember(Holdable(ScopedConnection(connection))));
  }
}

template <typename Owner, typename Fn, EnableIfConnectionOwner<Owner>>
Dispatcher::Connection Dispatcher::Connect(Fn&& handler, Owner owner) {
  return Connect(NodeHandle(), std::forward<Fn>(handler), owner);
}

template <typename Owner, typename Fn, EnableIfConnectionOwner<Owner>>
Dispatcher::Connection Dispatcher::Connect(NodeHandle node, Fn&& handler,
                                           Owner owner) {
  using FnType = typename std::remove_reference<Fn>::type;
  using EventType = decltype(ConnectHelper(&FnType::operator()));
  const HashValue type = EventTypeHelper::GetEventTypeHash<EventType>();

  return ConnectVariant(node, type,
                        EventHandlerVariantFromFn(std::forward<Fn>(handler)),
                        std::forward<Owner>(owner));
}

template <typename Owner, EnableIfConnectionOwner<Owner>>
Dispatcher::Connection Dispatcher::Connect(
    HashValue type, EventHandlerPropagationResult handler, Owner&& owner) {
  return ConnectVariant(type, EventHandlerVariant(std::move(handler)),
                        std::forward<Owner>(owner));
}

template <typename Owner, EnableIfConnectionOwner<Owner>>
Dispatcher::Connection Dispatcher::Connect(HashValue type,
                                           EventHandlerVoid handler,
                                           Owner&& owner) {
  return ConnectVariant(type, EventHandlerVariant(std::move(handler)),
                        std::forward<Owner>(owner));
}

template <typename Owner, EnableIfConnectionOwner<Owner>>
Dispatcher::Connection Dispatcher::Connect(
    NodeHandle node, HashValue type, EventHandlerPropagationResult handler,
    Owner&& owner) {
  return ConnectVariant(node, type, EventHandlerVariant(std::move(handler)),
                        std::forward<Owner>(owner));
}

template <typename Owner, EnableIfConnectionOwner<Owner>>
Dispatcher::Connection Dispatcher::Connect(NodeHandle node, HashValue type,
                                           EventHandlerVoid handler,
                                           Owner&& owner) {
  return ConnectVariant(node, type, EventHandlerVariant(std::move(handler)),
                        std::forward<Owner>(owner));
}

template <typename Owner, EnableIfConnectionOwner<Owner>>
Dispatcher::Connection Dispatcher::ConnectVariant(HashValue type,
                                                  EventHandlerVariant handler,
                                                  Owner&& owner) {
  return ConnectVariant(NodeHandle(), type, std::move(handler),
                        std::forward<Owner>(owner));
}

template <typename Owner, EnableIfConnectionOwner<Owner>>
Dispatcher::Connection Dispatcher::ConnectVariant(NodeHandle node,
                                                  HashValue type,
                                                  EventHandlerVariant handler,
                                                  Owner&& owner) {
  assert(type != 0);
  auto connection = ConnectImpl(node, type, std::move(handler),
                                ConnectionOwner(std::forward<Owner>(owner)));
  KeepAlive(owner, connection);
  return connection;
}

template <
    typename EventType, typename Owner,
    std::enable_if_t<std::is_constructible<ConnectionOwner, Owner>::value &&
                         std::is_base_of<Event, EventType>::value,
                     int>>
void Dispatcher::Disconnect(Owner&& owner) {
  Disconnect(NodeHandle(), EventTypeHelper::GetEventTypeHash<EventType>(),
             std::forward<Owner>(owner));
}

template <
    typename EventType, typename Owner,
    std::enable_if_t<std::is_constructible<ConnectionOwner, Owner>::value &&
                         std::is_base_of<Event, EventType>::value,
                     int>>
void Dispatcher::Disconnect(NodeHandle node, Owner&& owner) {
  Disconnect(node, EventTypeHelper::GetEventTypeHash<EventType>(),
             std::forward<Owner>(owner));
}

template <
    typename Owner,
    std::enable_if_t<std::is_constructible<ConnectionOwner, Owner>::value, int>>
void Dispatcher::Disconnect(HashValue type, Owner&& owner) {
  Disconnect(NodeHandle(), type, std::forward<Owner>(owner));
}

template <
    typename Owner,
    std::enable_if_t<std::is_constructible<ConnectionOwner, Owner>::value, int>>
void Dispatcher::Disconnect(NodeHandle node, HashValue type, Owner&& owner) {
  DisconnectImpl(node, type, ConnectionOwner(std::forward<Owner>(owner)));
}

template <
    typename Owner,
    std::enable_if_t<std::is_constructible<ConnectionOwner, Owner>::value, int>>
void Dispatcher::DisconnectAll(Owner owner) {
  DisconnectAll(ConnectionOwner(owner));
}

template <typename Fn>
Dispatcher::EventHandlerVariant Dispatcher::EventHandlerVariantFromFn(Fn&& fn) {
  using FnType = typename std::remove_reference<Fn>::type;
  using EventType = decltype(ConnectHelper(&FnType::operator()));
  using ReturnType = std::invoke_result_t<Fn, EventType>;

  static_assert(
      std::is_base_of_v<Event, EventType>,
      "Provided function must take a subclass of Event as a parameter.");

  if constexpr (std::is_same_v<ReturnType, void>) {
    return EventHandlerVoid(
        [fn = std::forward<Fn>(fn)](const Event& event) mutable {
          const EventType& casted_event = static_cast<const EventType&>(event);
          fn(casted_event);
        });
  } else {
    return EventHandlerPropagationResult(
        [fn = std::forward<Fn>(fn)](
            const Event& event) mutable -> PropagationResult {
          const EventType& casted_event = static_cast<const EventType&>(event);
          return fn(casted_event);
        });
  }
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_DISPATCHER_H_
