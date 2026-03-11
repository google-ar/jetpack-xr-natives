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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_WEB_VIEW_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_WEB_VIEW_H_

#include <cstdint>
#include <memory>
#include <string>

#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "core/math/vec.h"
#include "core/view/scripting/script_message_handler.h"

namespace imp::scripting {

inline constexpr float2 DEFAULT_LOCATION = {0.0f, 0.0f};
inline constexpr int2 DEFAULT_FULLSCREEN_DIMENSIONS = {-1, -1};

// An enum to control whether the web API is injected directly into the main
// page or into the first iframe found in the main page using the
// bridge/injection system to communicate between the two.
enum class InjectionTarget {
  kMainPage,
  kIFrame,
};

// Parameters to pass for WebView::Create.
struct WebViewParams {
  // Location of WebView in x, y pixels from top left corner.
  float2 location_px = DEFAULT_LOCATION;
  // Dimensions in width, height pixel values. Defaults to a fullscreen WebView.
  int2 dimensions_px = DEFAULT_FULLSCREEN_DIMENSIONS;
  std::string url;
  // Whether to inject the script into the main page or the first iframe.
  // Note: this parameter is only handled by WASM. All other platforms use
  // a WebView which serves the same function as an iFrame in terms of security.
  InjectionTarget injection_target = InjectionTarget::kMainPage;
};

// Base class for a platform-specific WebView.
class WebView {
 public:
  enum class State : uint8_t { kUnavailable, kAvailable };

  // Creates a WebView - must be implemented by subclasses per-platform.
  static std::unique_ptr<WebView> Create(
      ScriptMessageHandler& script_message_handler, const Context& context,
      const WebViewParams& params, BufferAccess injection_script);

  // TODO Separate this into platform-specific constructors
  // Attaches to an existing WebView to enable bi-directional communication
  // with it via an injection script.
  static std::unique_ptr<WebView> Create(
      ScriptMessageHandler& script_message_handler, const Context& context,
      void* web_view, BufferAccess injection_script);

  virtual ~WebView() {}

  // Gets the state of the webview, i.e. whether it has connected to JS yet.
  State GetState() const;

  // Loads the current injection script into the WebView.
  virtual void LoadInjectionScript() = 0;

  // Callback when the platform webview has been destroyed.
  void OnWebViewDestroyed();

 protected:
  WebView();

 private:
  State state_;
};

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_WEB_VIEW_H_
