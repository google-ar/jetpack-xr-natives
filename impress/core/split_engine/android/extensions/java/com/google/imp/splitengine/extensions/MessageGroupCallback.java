/*
 * Copyright 2025 Google LLC
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

package com.google.imp.splitengine.extensions;

import java.util.concurrent.Executor;

/** Implementation of the callback for message groups. */
public final class MessageGroupCallback {
  private final long clientId;
  private Executor mExecutor;

  public MessageGroupCallback(long clientId) {
    this.clientId = clientId;
  }

  // TODO: (broken link) - use long instead of int for messageGroupId.
  public void onMessageGroupComplete(int messageGroupId) {
    nOnMessageGroupComplete(clientId, messageGroupId);
  }

  // TODO: (broken link) - use long instead of int for messageGroupId.
  private static native void nOnMessageGroupComplete(long clientId, int messageGroupId);
}
