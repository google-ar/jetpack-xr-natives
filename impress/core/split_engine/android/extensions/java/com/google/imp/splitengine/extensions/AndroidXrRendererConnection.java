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

import android.os.ParcelFileDescriptor;
import android.os.RemoteException;
import android.os.SharedMemory;
import android.util.Log;
import android.view.Surface;
import androidx.annotation.Nullable;
import com.google.android.filament.proguard.UsedByNative;
import java.io.IOException;

/**
 * A connection to the Android XR split engine bridge service, providing methods for interacting
 * with shared memory and other rendering-related operations.
 */
@UsedByNative("split_engine_bridge.cc")
public final class AndroidXrRendererConnection implements IRendererConnection {
  private static final String TAG = "XrRendererConnection";

  @SuppressWarnings("UnnecessarilyFullyQualified")
  private final com.android.extensions.xr.splitengine.SystemRendererConnection
      mLibrarySystemRendererConnection;

  @UsedByNative("split_engine_bridge.cc")
  public AndroidXrRendererConnection(
      @SuppressWarnings("UnnecessarilyFullyQualified")
          com.android.extensions.xr.splitengine.SystemRendererConnection systemRendererConnection) {
    mLibrarySystemRendererConnection = systemRendererConnection;
  }

  @Nullable
  @Override
  @UsedByNative("split_engine_bridge.cc")
  public IBufferHandle registerBuffer(int fd, int bufferSizeBytes) {
    try {
      ParcelFileDescriptor pfd = ParcelFileDescriptor.fromFd(fd);
      SharedMemory sharedMemory = SharedMemory.fromFileDescriptor(pfd);
      return new BufferHandle(mLibrarySystemRendererConnection.registerBuffer(sharedMemory));
    } catch (IOException | RemoteException e) {
      return null;
    }
  }

  @Override
  @UsedByNative("split_engine_bridge.cc")
  public void processRegion(IBufferHandle bufferHandle, int offsetBytes, int regionLengthBytes)
      throws RemoteException {
    mLibrarySystemRendererConnection.processRegion(
        ((BufferHandle) bufferHandle).getLibraryBufferHandle(), offsetBytes, regionLengthBytes);
  }

  @Nullable
  @Override
  @UsedByNative("split_engine_bridge.cc")
  public Surface createExternalTextureSurface(long[] textureIds) {
    try {
      return mLibrarySystemRendererConnection.createExternalTextureSurface(textureIds);
    } catch (RemoteException e) {
      Log.e(TAG, "Failed to create external texture surface", e);
      return null;
    }
  }

  @Override
  @UsedByNative("split_engine_bridge.cc")
  public void setExternalTextureSurfaceSize(long textureId, int width, int height) {
    try {
      mLibrarySystemRendererConnection.setExternalTextureSurfaceSize(textureId, width, height);
    } catch (RemoteException e) {
      Log.e(TAG, "Failed to set external texture surface size", e);
      return;
    }
  }

  @Override
  @UsedByNative("split_engine_bridge.cc")
  public void sendRequest(byte[] data, RequestCallback callback) {
    try {
      mLibrarySystemRendererConnection.sendRequest(data, callback::onResult);
    } catch (RemoteException e) {
      Log.e(TAG, "Failed to send request", e);
      return;
    }
  }

  @Override
  @UsedByNative("split_engine_bridge.cc")
  public void addMessageGroupCallback(MessageGroupCallback callback) {
    mLibrarySystemRendererConnection.addMessageGroupCallback(callback::onMessageGroupComplete);
  }

  @Override
  @UsedByNative("split_engine_bridge.cc")
  public void close() {
    Log.d(TAG, "Closing connection.");
  }

  @SuppressWarnings("UnnecessarilyFullyQualified")
  @UsedByNative("split_engine_bridge.cc")
  public com.android.extensions.xr.splitengine.SystemRendererConnection getConnectionHandle() {
    return mLibrarySystemRendererConnection;
  }

  /** A handle to a buffer instance. */
  @UsedByNative("split_engine_bridge.cc")
  public static final class BufferHandle implements IBufferHandle {
    private com.android.extensions.xr.splitengine.BufferHandle mBufferHandle = null;

    @UsedByNative("split_engine_bridge.cc")
    public BufferHandle(com.android.extensions.xr.splitengine.BufferHandle bufferHandle) {
      mBufferHandle = bufferHandle;
    }

    @UsedByNative("split_engine_bridge.cc")
    public com.android.extensions.xr.splitengine.BufferHandle getLibraryBufferHandle() {
      return mBufferHandle;
    }
  }
}
