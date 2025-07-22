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

import android.os.IBinder;
import android.view.Surface;
import java.util.concurrent.Executor;

/**
 * A connection to the system renderer service, providing methods for interacting with shared memory
 * and other rendering-related operations.
 */
public final class PrototypeRendererConnectionNdk implements IRendererConnection {

  private long mNativeBridgeHandle;
  private final Executor workScheduler;

  public PrototypeRendererConnectionNdk(IBinder serviceBinder, Executor workScheduler) {
    mNativeBridgeHandle = nCreateBridge(serviceBinder);
    this.workScheduler = workScheduler;
  }

  /**
   * Registers a shared memory buffer with the bridge and returns a handle to the shared memory
   * buffer.
   */
  @Override
  public IBufferHandle registerBuffer(int fd, int bufferSizeBytes) {
    return new BufferHandle(nRegisterBuffer(mNativeBridgeHandle, fd, bufferSizeBytes));
  }

  @Override
  public void processRegion(IBufferHandle bufferHandle, int offsetBytes, int regionLengthBytes) {
    nProcessRegion(
        mNativeBridgeHandle,
        ((BufferHandle) bufferHandle).getNativeHandle(),
        offsetBytes,
        regionLengthBytes);
  }

  @Override
  public Surface createExternalTextureSurface(long[] textureIds) {
    return nCreateExternalTextureSurface(mNativeBridgeHandle, textureIds);
  }

  @Override
  public void setExternalTextureSurfaceSize(long textureId, int width, int height) {
    nSetExternalTextureSurfaceSize(mNativeBridgeHandle, textureId, width, height);
  }

  @Override
  public void sendRequest(byte[] data, RequestCallback callback) {
    nSendRequest(mNativeBridgeHandle, data, callback.getNativeHandle());
  }

  @Override
  public void addMessageGroupCallback(MessageGroupCallback callback) {
    nRegisterMessageGroupCallback(mNativeBridgeHandle, callback);
  }

  @Override
  public void close() {
    if (mNativeBridgeHandle != 0) {
      nDestroyBridge(mNativeBridgeHandle);
    }
    mNativeBridgeHandle = 0;
  }

  /** A handle to a buffer instance. */
  private static final class BufferHandle implements IBufferHandle {
    private final long mNativeBufferHandle;

    public BufferHandle(long nativeBufferHandle) {
      mNativeBufferHandle = nativeBufferHandle;
    }

    public long getNativeHandle() {
      return mNativeBufferHandle;
    }
  }

  // JNI methods.

  private static native long nCreateBridge(IBinder serviceBinder);

  private static native void nRegisterMessageGroupCallback(
      long nativeBridgeHandle, Object callback);

  private static native long nRegisterBuffer(long nativeBridgeHandle, int fd, int bufferSizeBytes);

  private static native void nProcessRegion(
      long nativeBridgeHandle, long nativeBufferHandle, int offsetBytes, int regionLengthBytes);

  private static native Surface nCreateExternalTextureSurface(
      long nativeBridgeHandle, long[] textureIds);

  private static native void nSetExternalTextureSurfaceSize(
      long nativeBridgeHandle, long textureId, int width, int height);

  private static native void nSendRequest(
      long nativeBridgeHandle, byte[] data, long nativeRequestCallbackHandle);

  private static native void nDestroyBridge(long nativeBridgeHandle);
}
