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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_CLIENT_API_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_CLIENT_API_H_

#include <functional>
#include <memory>

#include "absl/strings/string_view.h"
#include "core/view/framework/view.h"

// The build rules in imp_view.bzl are designed to wrap a client library that
// implements this API.
namespace imp {
namespace client_api {

using CreateViewFn = std::function<std::unique_ptr<View>(absl::string_view)>;
using CreateViewForIdentifierFn = std::function<std::unique_ptr<View>()>;

// Can be called once by each application integrating with the Impress framework
// to tell the platform code which subclass of View to instantiate.
//
// The "identifier" parameter can be used to switch between different subclasses
// of View.
//
// This will throw an error if it's called more than once.
bool SetCreateViewFn(CreateViewFn create_view_fn);

// Can be called by users of the Impress framework to tell the platform code
// which subclass of View to instantiate for a particular identifier string.
//
// This is similar to the above method, except that it can safely be called
// multiple times. This makes it easy for multiple libraries using Impress
// within the same application to specify their View subclass within their own
// library without needing to centralize the call to SetCreateViewFn in one
// place.
//
// If both versions of SetCreateViewFn are called, then this version will be
// prioritized. The other version will be used as a fallback if nothing has been
// registered for this specific identifier.
//
// This will throw an error if it's called for the same identifier more than
// once.
bool SetCreateViewFn(absl::string_view identifier,
                     CreateViewForIdentifierFn create_view_for_identifier_fn);

// Called by platform integration code to instantiate the view.
// This makes it possible for a developer to write a cross-platform impress app.
// without needing to write platform specific boilerplate to instantiate the
// correct view on each platform.
std::unique_ptr<View> CreateView(absl::string_view identifier);

// Returns true if the app is being built in "sandbox" mode. This will be set
// to true automatically by the "myapp_sandbox" build target (which always
// exists if your app is tagged to support the desktop platform).
// Sandbox builds do not call the View::Setup() function, instead calling
// View::SetupSandbox(). Your app should register all Components and assets
// in View::RegisterComponents(), which is called in both app contexts.
bool GetIsAppSandboxTarget();

// Statically makes the current build a "sandbox" build. Returns true.
bool MakeAppSandboxTarget();

}  // namespace client_api
}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_CLIENT_API_H_
