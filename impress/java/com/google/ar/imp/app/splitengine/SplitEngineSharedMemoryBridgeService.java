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

package com.google.ar.imp.app.splitengine;

import android.app.Service;
import android.content.Intent;
import android.os.IBinder;
import android.util.Log;

/**
 * Accesses the Impress Split Engine buffer service.
 *
 * @hide
 */
public class SplitEngineSharedMemoryBridgeService extends Service {
  private static final String TAG = SplitEngineSharedMemoryBridgeService.class.getSimpleName();

  private IBinder mBinder;

  // Loads the 'split_engine_shared_memory_bridge_lib' library on startup.
  static {
    // TODO: Allow naming of the library to be configurable by the user.
    System.loadLibrary("imp_view_jni");
  }

  @Override
  public IBinder onBind(Intent intent) {
    Log.d(TAG, "SplitEngineSharedMemoryBridgeService got a request to bind.");

    return mBinder;
  }

  @Override
  public int onStartCommand(final Intent intent, final int flags, final int startId) {
    Log.d(TAG, "SplitEngineSharedMemoryBridgeService got a request to start.");

    if (intent != null) {
      // The process which hosts the Impress backend instance provides the View as well as the
      // foreground executor that are needed to start the Split Engine bridge service. Both handles
      // are passed by intent extras and will be converted to their respective native type over JNI.
      long impView = intent.getLongExtra("impView", -1);
      long executor = intent.getLongExtra("executor", -1);

      if (impView == -1 || executor == -1) {
        Log.e(TAG, "Received invalid intent extras to start the service.");
        return START_NOT_STICKY;
      }

      mBinder = nCreateServiceBinder(impView, executor);

      if (mBinder == null) {
        Log.e(TAG, "Could not create service binder.");
      }
    } else {
      Log.e(TAG, "Received an invalid intent to start the service.");
    }

    return START_NOT_STICKY;
  }

  // LINT.IfChange(service)
  /**
   * A native method that is implemented by the 'imp_view_jni' native library, which sets up the
   * SplitEngineSharedMemoryBridgeService.
   */
  public native IBinder nCreateServiceBinder(long viewHandle, long executorHandle);
  // LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/app/splitengine/SplitEngineSharedMemoryBridgeService.java:service)
}
