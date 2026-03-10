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
package com.google.ar.imp.materialcompiler;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.IBinder;
import android.os.ParcelFileDescriptor;
import android.os.RemoteException;
import android.util.Log;
import androidx.annotation.VisibleForTesting;
import com.google.ar.imp.materialcompiler.aidl.IMaterialCompilerService;
import java.io.IOException;

/** Java side client that kicks off runtime material compilation. */
public final class MaterialCompilerClient implements ServiceConnection {
  private static final String TAG = MaterialCompilerClient.class.getSimpleName();
  // TODO: Allow override; Also have this in a shared place since it's used in multiple
  // locations.
   static final String DEFAULT_LIBRARY_NAME = "imp_view_jni";

  private IBinder materialCompilerServiceBinder;
   IMaterialCompilerService materialCompilerService;
  private final Context context;
  private String resolvedLibraryName;
  private final long nativeHandle;

  private enum State {
    UNBOUND, // Initial state, before `startService` is called.
    PENDING, // `startService` is called, and the service is pending to be bound.
    CONNECTED, // Service is connected.
    CLOSED // `close` is called and it's permanently shut down.
  }

  private volatile State state = State.UNBOUND;

  private final FileDescriptorFactory fileDescriptorFactory;
  private final NativeCallHandler nativeCallHandler;

  public MaterialCompilerClient(Context context, long nativeHandle, String nativeLibraryOverride) {
    this.context = context;
    this.resolvedLibraryName = nativeLibraryOverride;
    this.nativeHandle = nativeHandle;
    this.fileDescriptorFactory = new FileDescriptorFactoryImpl();
    this.nativeCallHandler = new NativeCallHandlerImpl();
  }

  public MaterialCompilerClient(Context context, long nativeHandle) {
    this(context, nativeHandle, DEFAULT_LIBRARY_NAME);
  }

  // Constructor that allows to override ParcelFileDescriptor for testing.
  
  MaterialCompilerClient(
      Context context,
      long nativeHandle,
      FileDescriptorFactory fileDescriptorFactory,
      NativeCallHandler nativeCallHandler,
      String nativeLibraryOverride) {
    this.context = context;
    this.resolvedLibraryName = nativeLibraryOverride;
    this.nativeHandle = nativeHandle;
    this.fileDescriptorFactory = fileDescriptorFactory;
    this.nativeCallHandler = nativeCallHandler;
  }

  // This is called from the C++ side to start the material compiler service.
  // The connection future will return the client file descriptor when the connection is
  // successfully established.
  public void startService() {
    synchronized (this) {
      if (state != State.UNBOUND) {
        return;
      }
      state = State.PENDING;
    }
    Intent intent = new Intent(context, MaterialCompilerService.class);

    String packageName = context.getPackageName();
    // MaterialCompilerService is hosted in SpaceFlinger (See its AndroidManifest). In AndroidXR,
    // we send the Intent in the SystemUI context, then we need to explicitly target SpaceFlinger's
    // package. Note that this is only relevant to AndroidXR.
    // TODO: Find a better way to override the package and the native library names.
    if (packageName.equals("com.android.systemui")) {
      packageName = "com.android.spaceflinger";
      // The name comes from
      // (broken link)
      this.resolvedLibraryName = "sysui_jni_soong";
    }
    intent.setClassName(packageName, MaterialCompilerService.class.getName());

    boolean bindServiceResult =
        context.bindService(
            intent,
            this,
            Context.BIND_AUTO_CREATE | Context.BIND_IMPORTANT | Context.BIND_ADJUST_WITH_ACTIVITY);
    if (!bindServiceResult) {
      nativeCallHandler.onServiceConnected(nativeHandle, 0, "Failed to bind service");
    }
  }

  // This can be called from the native side, or from the service disconnect callback.
  public synchronized void close() {
    // We are already closed or closing, do nothing.
    if (state == State.CLOSED) {
      return;
    }
    state = State.CLOSED;

    nativeCallHandler.close(nativeHandle);

    if (materialCompilerService != null) {
      try {
        // This will also close the native service from the service side.
        materialCompilerService.close();
        materialCompilerService = null;
      } catch (RemoteException e) {
        Log.e(TAG, "failed to destroy material compiler service " + e.getMessage());
      }
    }

    if (materialCompilerServiceBinder != null) {
      context.unbindService(this);
      materialCompilerServiceBinder = null;
    }
  }

  // This is called once the service is connected by the `startService` call above.
  @Override
  public void onServiceConnected(ComponentName componentName, IBinder binder) {
    // If the client is already closed or if the connection is already established, but we get
    // onServiceConnected callback again, destroy the service immediately.
    //
    // This is generally unlikely to happen but it's possible in the following scenario:
    // C++ side creates C++ and Java clients, and the Java client (this) kicks off the service.
    // Before the service is connected, C++ side can call `close` (due to some error). Then later,
    // we receive `onServiceConnected` callback, but at this point we don't a the valid client,
    // so we need to skip the setup, otherwise we will get a zombie client.
    synchronized (this) {
      if (state != State.PENDING) {
        // Immediately destroy the service that just arrived.
        try {
          IMaterialCompilerService.Stub.asInterface(binder).close();
        } catch (RemoteException e) {
          Log.e(TAG, "Failed to destroy late-arriving service connection.", e);
        }
        return; // Do not proceed with setup.
      }
      // Otherwise we are on the happy path.
      state = State.CONNECTED;
    }

    materialCompilerService = IMaterialCompilerService.Stub.asInterface(binder);
    materialCompilerServiceBinder = binder;

    // Now that we have Android service bound, we create native service.
    try {
      // This can throw an IOException.
      // fds[0] is for client fds[1] is for service.
      ParcelFileDescriptor[] fds = fileDescriptorFactory.createReliableSocketPair();

      // This can throw a RemoteException.
      materialCompilerService.create(fds[1], resolvedLibraryName);

      nativeCallHandler.onServiceConnected(nativeHandle, fds[0].detachFd(), /* errorMessage= */ "");
    } catch (IOException | RemoteException e) {
      Log.e(TAG, "Failed to finish setting up the service: " + e.getMessage());
      nativeCallHandler.onServiceConnected(nativeHandle, 0, e.getMessage());
    }
  }

  @Override
  public void onServiceDisconnected(ComponentName componentName) {
    close();
  }

  /** Creates a socket pair. */
  interface FileDescriptorFactory {
    ParcelFileDescriptor[] createReliableSocketPair() throws IOException;
  }

  static class FileDescriptorFactoryImpl implements FileDescriptorFactory {
    @Override
    public ParcelFileDescriptor[] createReliableSocketPair() throws IOException {
      return ParcelFileDescriptor.createReliableSocketPair();
    }
  }

  /** Handles native calls. */
  interface NativeCallHandler {
    void onServiceConnected(long nativeHandle, int fd, String errorMessage);

    void close(long nativeHandle);
  }

  static class NativeCallHandlerImpl implements NativeCallHandler {
    @Override
    public void onServiceConnected(long nativeHandle, int fd, String errorMessage) {
      nOnServiceConnected(nativeHandle, fd, errorMessage);
    }

    @Override
    public void close(long nativeHandle) {
      if (nativeHandle != 0) {
        nClose(nativeHandle);
      }
    }
  }

  // LINT.IfChange(native_api)
  private static native void nClose(long nativeHandle);

  private static native void nOnServiceConnected(long nativeHandle, int fd, String errorMessage);
  // LINT.ThenChange(//depot/google3/third_party/impress/core/materials/compiler/material_compiler_client_jni.cc:native_api)
}
