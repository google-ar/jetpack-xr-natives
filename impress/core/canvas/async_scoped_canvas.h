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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_SCOPED_CANVAS_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_SCOPED_CANVAS_H_

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/canvas/scoped_canvas.h"

namespace imp {

// A ScopedCanvas whose draw commands may not complete synchronously.
struct AsyncScopedCanvas : ScopedCanvas {
  // Does any asynchronous work necessary to be able to apply the pending
  // draw commands to the texture synchronously on destruction.
  virtual Future<absl::Status> PrepareToUpdateTexture() = 0;

  // Whether this implementation of an AsyncScopedCanvas can always apply draw
  // commands synchronously to the texture on destruction without calling
  // PrepareToUpdateTexture. If true, clients can assume PrepareToUpdateTexture
  // will always resolve synchronously. If false, PrepareToUpdateTexture may
  // resolve asynchronously or synchronously depending on if there is work
  // to do.
  virtual bool SupportsSynchronousTextureUpdate() const = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_SCOPED_CANVAS_H_
