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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SCENE_LOAD_SCENE_VISITOR_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SCENE_LOAD_SCENE_VISITOR_H_

#include <tuple>

#include "core/common/invocable.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node.h"
#include "core/view/base_view.h"

namespace imp {

// Visitor used to hook into the process of loading a scene when calling
// SceneSystem::LoadScene to modify & inspect the deserialized state of a
// component prior to component's being Setup.
//
// This makes it possible to dynamically override fields loaded from a .isf
// file.
//
// For example, you could override what gltf file is loaded like this:
//
//   std::string some_gltf_url;
//   LoadSceneVisitor visitor;
//   visitor.OnVisit([some_gltf_url](NodeHandle node, GltfState& state) {
//     state.asset = some_gltf_url;
//   });
//
//   GetSceneSystem().LoadScene(kFooBarIsf, std::move(visitor));
// TODO: Explore sharing implementation between ParseMessageVisitor
// and LoadSceneVisitor.
class LoadSceneVisitor : public BaseStateVisitor {
 public:
  LoadSceneVisitor() {}

  LoadSceneVisitor(const LoadSceneVisitor&) = delete;
  LoadSceneVisitor(LoadSceneVisitor&&) = default;

  LoadSceneVisitor& operator=(const LoadSceneVisitor&) = delete;
  LoadSceneVisitor& operator=(LoadSceneVisitor&&) = default;

  // Called automatically when a scene is loading. Takes a type-erased state for
  // a component, and the node that the component was attached to.
  absl::optional<Future<absl::Status>> Accept(
      NodeHandle node, HashValue state_type_hash, void* erased_state,
      bool should_enable_component) override;

  // Registers a functor to be called just before a component's Setup method is
  // called when loading a Scene from a .isf file.
  //
  // The functor's first parameter should always be a NodeHandle, and the second
  // parameter should be a reference to the type of State.
  //
  // The functor will be called automatically for every component's state in the
  // .isf file that matches the type of state the functor takes as a parameter.
  // The node passed into the functor will be the node that the component's
  // state is attached to.
  //
  // The functor is always called prior to the component's Setup function being
  // called. Therefore, this can be used to modify the deserialized state before
  // any Setup work (async or otherwise) is done.
  //
  // Example:
  //
  //   LoadSceneVisitor visitor;
  //   visitor.OnVisit([some_gltf_url](NodeHandle node, GltfState& state) {
  //     state.asset = some_gltf_url;
  //   });
  template <typename Fn>
  void OnVisit(Fn fn);

  // Registers a functor to be called just before a component's Setup method is
  // called when loading a Scene from a .isf file.
  //
  // This is exactly like the other overload of OnVisit, except that the result
  // returned by Fn will be passed into the component's Setup method, allowing
  // you to control what parameters are passed into the component's Setup
  // method.
  //
  // Example:
  //
  //   LoadSceneVisitor visitor;
  //   visitor.OnVisit([](NodeHandle node, FooState& state) {
  //     // Will call this Setup signature:
  //     //   Foo::Setup(int);
  //     return 10;
  //   });
  //
  // To call a Setup method taking multiple arguments, you can return an
  // std::tuple, then each element of the tuple is passed into the component's
  // setup method:
  //
  //   LoadSceneVisitor visitor;
  //   visitor.OnVisit([](NodeHandle node, FooState& state) {
  //     // Will call this Setup signature:
  //     //   Foo::Setup(int, std::string);
  //     // Or any other Setup accepting those types:
  //     //   Foo::Setup(int, absl::string_view);
  //     return std::make_tuple(10, "Hello");
  //   });
  //
  // To use a different overload of Setup for different component instances in
  // the .isf file, you can return type is an absl::variant:
  //
  //   LoadSceneVisitor visitor;
  //   visitor.OnVisit([](NodeHandle node, FooState& state)
  //     -> absl::variant<int, std::tuple<int, std::string> {
  //         if (node->GetName() == "Bar") {
  //           // Call Foo::Setup(int) for the node named "Bar".
  //           return 10;
  //         }
  //
  //       // For any other node, call Foo::Setup(int, std::string).
  //       return std::make_tuple(20, "Hello");
  //   });
  //
  // If nothing is returned, then the default parameterless Setup is used.
  // You can also return absl::monostate in an absl::variant to indicate the
  // default Setup should be used.
  template <typename T, typename Fn>
  void OnVisit(Fn fn);

 private:
  // Helper function declaration that is used to extract the type of the state
  // parameter of the functor passed into OnVisit.
  template <typename Fn, typename Arg, typename Ret>
  static Arg GetStateTypeHelper(Ret (Fn::*)(NodeHandle, Arg&) const);

  // Mutable helper function declaration that is used to extract the type of the
  // state parameter of the functor passed into OnVisit.
  template <typename Fn, typename Arg, typename Ret>
  static Arg GetStateTypeHelper(Ret (Fn::*)(NodeHandle, Arg&));

  // Helper function declaration that is used to extract the return type of the
  // functor passed into OnVisit.
  template <typename Fn, typename Arg, typename Ret>
  static Ret GetReturnTypeHelper(Ret (Fn::*)(NodeHandle, Arg&) const);

  // Helper function declaration that is used to extract the return type of the
  // functor passed into OnVisit.
  template <typename Fn, typename Arg, typename Ret>
  static Ret GetReturnTypeHelper(Ret (Fn::*)(NodeHandle, Arg&));

  using VisitFn = Invocable<absl::optional<Future<absl::Status>>(
      NodeHandle node, void* erased_state, bool should_enable_component)>;
  tsl::robin_map<HashValue, VisitFn> visit_functions_;
};

template <typename Fn>
void LoadSceneVisitor::OnVisit(Fn fn) {
  // Deduce the State type from fn's function signature.
  using FnType = typename std::remove_reference<Fn>::type;
  using StateType = decltype(GetStateTypeHelper(&FnType::operator()));
  using ReturnType = decltype(GetReturnTypeHelper(&FnType::operator()));
  static_assert(std::is_void_v<ReturnType>,
                "The type of component must be specified as a template "
                "parameter to OnVisit to specify a non-void return type.");

  // Wrap fn in a function that takes the type-erased version of the State and
  // converts it back into the concrete type. Register the wrapped fn based on
  // the type hash of StateType. This is needed because all of the
  // SceneComponentDeserializer::Handler's used to register components for
  // deserialization with SceneSystem are type-erased. This is similar to the
  // technique used to implement the Dispatcher.
  constexpr HashValue state_type_hash = type_traits::kTypeHash<StateType>;
  visit_functions_[state_type_hash] = [fn = std::move(fn)](
                                          NodeHandle node, void* erased_state,
                                          bool should_enable_component)
      -> absl::optional<Future<absl::Status>> {
    StateType* state = static_cast<StateType*>(erased_state);
    fn(node, *state);
    return absl::nullopt;
  };
}

template <typename T, typename Fn>
void LoadSceneVisitor::OnVisit(Fn fn) {
  using FnType = typename std::remove_reference<Fn>::type;
  using StateType = decltype(GetStateTypeHelper(&FnType::operator()));
  using ReturnType = decltype(GetReturnTypeHelper(&FnType::operator()));
  static_assert(std::is_same_v<StateType, typename T::IsfInfo::StateT>);

  constexpr HashValue state_type_hash = type_traits::kTypeHash<StateType>;
  visit_functions_[state_type_hash] = [fn = std::move(fn)](
                                          NodeHandle node, void* erased_state,
                                          bool should_enable_component) mutable
      -> absl::optional<Future<absl::Status>> {
    StateType* state = static_cast<StateType*>(erased_state);
    if constexpr (std::is_void_v<ReturnType>) {
      // Fn returns nothing, so we just call it and return nullopt.
      fn(node, *state);
      return absl::nullopt;
    } else {
      // Fn has a return type, so we call it and hold onto the returned object
      // so that we can use it to invoke the correct overload of the component's
      // Setup method based on the returned object.
      auto fn_result = fn(node, *state);
      using FnResult = decltype(fn_result);

      // Helper used to call Setup with the arguments pased in.
      auto apply_fn = [node, should_enable_component](auto&&... args) mutable {
        return shared_isf_info_handlers::Setup<T, decltype(args)...>(
            node.GetEntity(), &node->GetView().GetComponentManager(),
            should_enable_component, std::forward<decltype(args)>(args)...);
      };

      if constexpr (type_traits::IsTemplateType<FnResult, std::tuple>::value) {
        // Fn returned a tuple, so apply each element of the tuple to the
        // arguments in Setup.
        return std::apply(apply_fn, std::move(fn_result));
      } else if constexpr (type_traits::IsTemplateType<FnResult,
                                                       absl::variant>::value) {
        // Fn returned a variant, so visit each possible element of the variant
        // and correctly apply it to call Setup.
        return std::visit(
            [&apply_fn](
                auto arg) mutable -> absl::optional<Future<absl::Status>> {
              using Arg = decltype(arg);
              if constexpr (std::is_same_v<Arg, absl::monostate>) {
                // Variant contains a monostate indicating the default Setup
                // should be called, so we just return nullopt.
                return absl::nullopt;
              } else if constexpr (type_traits::IsTemplateType<
                                       Arg, std::tuple>::value) {
                // Variant contains a tuple, apply it.
                return std::apply(apply_fn, std::move(arg));
              } else {
                // Variant contains a single param, so try to call single
                // argument Setup function.
                return apply_fn(std::move(arg));
              }
            },
            std::move(fn_result));
      } else {
        // Result was a single param, try to call a single argument Setup
        // function.
        return apply_fn(std::move(fn_result));
      }
    }
  };
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_SCENE_LOAD_SCENE_VISITOR_H_
