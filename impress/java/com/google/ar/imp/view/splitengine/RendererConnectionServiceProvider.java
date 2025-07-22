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

package com.google.ar.imp.view.splitengine;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.pm.PackageManager;
import android.os.IBinder;
import android.os.RemoteException;
import android.util.Log;
import androidx.annotation.Nullable;
import androidx.concurrent.futures.CallbackToFutureAdapter;
import com.android.extensions.xr.XrExtensions;
import com.google.common.util.concurrent.ListenableFuture;
import com.google.imp.splitengine.extensions.AndroidXrRendererConnection;
import com.google.imp.splitengine.extensions.IRendererConnection;
import com.google.imp.splitengine.extensions.PrototypeRendererConnectionNdk;
import java.time.Duration;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executor;

/** Helper class to abstract retrieving a split engine bridge service binder. */
public class RendererConnectionServiceProvider implements ServiceConnection {

  public static final Duration WAIT_TIME_FOR_RENDERER_TO_START = Duration.ofSeconds(1);

  private static final String TAG = RendererConnectionServiceProvider.class.getSimpleName();

  private final ListenableFuture<IRendererConnection> bridgeFuture;
  private CallbackToFutureAdapter.Completer<IRendererConnection> bridgeCompleter;
  private final Executor frameSchedulerExecutor;
  private boolean isRunningOnPhone;
  private final XrExtensions xrExtensions;

  interface RendererConnectionReadyCallback {
    void onSystemRendererConnectionReady(IRendererConnection bridge);
  }

  /**
   * @param frameSchedulerExecutor The frameSchedulerExecutor to use for the callback.
   */
  public RendererConnectionServiceProvider(
      Executor frameSchedulerExecutor, @Nullable XrExtensions xrExtensions) {
    this.frameSchedulerExecutor = frameSchedulerExecutor;
    this.xrExtensions = xrExtensions;
    this.bridgeFuture =
        CallbackToFutureAdapter.getFuture(
            completer -> {
              this.bridgeCompleter = completer;
              return "SystemRendererConnectionServiceProvider.initializeService";
            });
  }

  @Override
  public void onServiceConnected(ComponentName componentName, IBinder iBinder) {
    Log.d(TAG, "Bound to SplitEngineSharedMemoryBridgeService.");

    IRendererConnection bridge =
        new PrototypeRendererConnectionNdk(iBinder, frameSchedulerExecutor);
    this.bridgeCompleter.set(bridge);
  }

  @Override
  public void onServiceDisconnected(ComponentName componentName) {
    // This can only happen on the phone, which is used for debugging / testing, so not a critical
    // failure.
    Log.e(
        TAG,
        "Application to no longer bound to SplitEngineSharedMemoryBridgeService. This is"
            + " unrecoverable.");
  }

  // Execute the provided callback when the bridge is ready.
  // On XROS, this is synchronous and will execute immediately on the calling thread.
  // On the phone (due to constraints of running the renderer as a bound service) the callback will
  // happen on the provided executor after the main thread returns.
  public void onBridgeReady(RendererConnectionReadyCallback bridgeCallback) {
    bridgeFuture.addListener(
        () -> {
          try {
            bridgeCallback.onSystemRendererConnectionReady(bridgeFuture.get());
          } catch (InterruptedException | ExecutionException e) {
            Log.e(TAG, "Failed to get split engine bridge. This should never happen.");
            throw new IllegalStateException("Unable to initialize Impress API", e);
          }
        },
        isRunningOnPhone ? frameSchedulerExecutor : Runnable::run);
  }

  @SuppressWarnings("InlinedApi")
  public void initializeService(Context context, IBinder serviceBinder) {
    Log.d(TAG, "Initialize the SplitEngineSharedMemoryBridgeService.");

    PackageManager pm = context.getPackageManager();
    if (serviceBinder != null) {
      isRunningOnPhone = false;
      IRendererConnection bridge =
          new PrototypeRendererConnectionNdk(serviceBinder, frameSchedulerExecutor);
      this.bridgeCompleter.set(bridge);
    } else if (pm.hasSystemFeature("android.software.xr.api.spatial")
        || pm.hasSystemFeature("android.software.xr.api.openxr")) {
      isRunningOnPhone = false;
      Log.d(TAG, "Running in XR.");

      try {
        AndroidXrRendererConnection bridge =
            new AndroidXrRendererConnection(
                xrExtensions.createSystemRendererConnection(
                    (Activity) context, frameSchedulerExecutor));

        this.bridgeCompleter.set(bridge);
      } catch (RemoteException e) {
        Log.e(TAG, "Failed to create system renderer connection.", e);
        this.bridgeCompleter.setException(e);
      }
    } else {
      isRunningOnPhone = true;
      Log.d(TAG, "Running on phone.");

      // Launch the phone renderer app
      Intent rendererIntent =
          context
              .getPackageManager()
              .getLaunchIntentForPackage("com.google.ar.imp.app.splitengine");

      rendererIntent.setFlags(Intent.FLAG_ACTIVITY_LAUNCH_ADJACENT | Intent.FLAG_ACTIVITY_NEW_TASK);

      context.startActivity(rendererIntent);

      Intent serviceIntent = new Intent();
      serviceIntent.setClassName(
          "com.google.ar.imp.app.splitengine",
          "com.google.ar.imp.app.splitengine.SplitEngineSharedMemoryBridgeService");

      // TODO: We need to give the renderer some time to start. We should be able to
      // make this more robust on the service side and remove the delay.
      try {
        Thread.sleep(WAIT_TIME_FOR_RENDERER_TO_START.toMillis());
      } catch (InterruptedException e) {
        Log.w(TAG, "Interrupted sleep waiting for renderer to start.");
      }

      context.bindService(
          serviceIntent, RendererConnectionServiceProvider.this, Context.BIND_AUTO_CREATE);
    }
  }
}
