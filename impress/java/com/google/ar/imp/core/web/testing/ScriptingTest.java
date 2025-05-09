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

package com.google.ar.imp.core.web.testing;

import android.os.Handler;
import android.os.Looper;
import com.google.android.filament.proguard.UsedByNative;

/** Helper functions for moving Android View initialization and tear down onto main thread. */
public final class ScriptingTest {
  @UsedByNative("scripting_test.cc")
  public static void initTestView(long nativeTestHandle, long contextHandle) {
    new Handler(Looper.getMainLooper()).post(() -> nInitTestView(nativeTestHandle, contextHandle));
  }

  @UsedByNative("scripting_test.cc")
  public static void tearDownView(long nativeTestHandle) {
    new Handler(Looper.getMainLooper()).post(() -> nTearDownView(nativeTestHandle));
  }

  private static native void nInitTestView(long testHandle, long contextHandle);

  private static native void nTearDownView(long testHandle);

  private ScriptingTest() {}
}
