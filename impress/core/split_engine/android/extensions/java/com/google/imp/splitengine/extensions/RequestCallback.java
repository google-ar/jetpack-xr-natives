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

import com.google.android.filament.proguard.UsedByNative;

/** Implementation of the callback for requests. */
@UsedByNative("split_engine_bridge.cc")
public final class RequestCallback {
  private final long nativeRequestCallbackHandle;

  @UsedByNative("split_engine_bridge.cc")
  public RequestCallback(long nativeRequestCallbackHandle) {
    this.nativeRequestCallbackHandle = nativeRequestCallbackHandle;
  }

  @UsedByNative("split_engine_bridge.cc")
  long getNativeHandle() {
    return nativeRequestCallbackHandle;
  }

  @UsedByNative("split_engine_bridge.cc")
  public void onResult(byte[] response) {
    nOnResult(nativeRequestCallbackHandle, response);
  }

  private static native void nOnResult(long nativeRequestCallbackHandle, byte[] response);
}
