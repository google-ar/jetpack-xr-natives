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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_SCRIPTING_SYSTEM_WEB_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_SCRIPTING_SYSTEM_WEB_H_

#include <memory>

#include "core/common/log.h"
#include "core/common/buffer_access.h"
#include "core/common/resource_helpers.h"
#include "core/scripting/scripting_system.h"
#include "core/scripting/web/web_view.h"
#include "core/view/base_view.h"
#include "javascript/core/imp_web_js_embed.h"

namespace imp::scripting {

static BufferAccess LoadWebScript() {
  BufferAccess script;

  // If IMP_WEB_JS_DEBUG, do not inject.
  // Use with TypeScript devserver for JS edit-refresh debugging.
#ifndef IMP_WEB_JS_DEBUG
  RegisterPackagedResources(imp_web_js_embed_create());
  if (auto status = LoadPackagedFile("web.js", &script); !status.ok()) {
    IMP_LOG(imp::FATAL) << status;
  }
#endif

  return script;
}

static std::unique_ptr<ScriptingSystem> CreateScriptingSystemWeb(
    BaseView& view, const WebViewParams& params) {
  return std::make_unique<ScriptingSystem>(view.GetContext(), &view, params,
                                           LoadWebScript());
}

static std::unique_ptr<ScriptingSystem> CreateScriptingSystemWeb(
    BaseView& view, void* external_web_view) {
  return std::make_unique<ScriptingSystem>(view.GetContext(), &view,
                                           external_web_view, LoadWebScript());
}

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_SCRIPTING_SYSTEM_WEB_H_
