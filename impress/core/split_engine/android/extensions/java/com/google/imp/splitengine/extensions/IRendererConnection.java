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

import android.os.RemoteException;
import android.view.Surface;

/** Interface for a connection to the system renderer. */
public interface IRendererConnection {
  /**
   * Registers a shared memory buffer with the bridge and returns a handle to the shared memory
   * buffer.
   */
  default IBufferHandle registerBuffer(int fd, int bufferSizeBytes) {
    throw new UnsupportedOperationException();
  }

  /** Processes a region of a registered buffer specified by offset and bytes. */
  default void processRegion(IBufferHandle bufferHandle, int offsetBytes, int regionLengthBytes)
      throws RemoteException {
    throw new UnsupportedOperationException();
  }

  /** Creates a texture surface bound to the given external texture id. */
  default Surface createExternalTextureSurface(long[] textureIds) {
    throw new UnsupportedOperationException();
  }

  /** Sets the size of an external texture surface bound to the given texture id. */
  default void setExternalTextureSurfaceSize(long textureId, int width, int height) {
    throw new UnsupportedOperationException();
  }

  /** Sends a flatbuffer request to the backend with a handler for a flatbuffer response. */
  default void sendRequest(byte[] data, RequestCallback callback) {
    throw new UnsupportedOperationException();
  }

  /** Adds a message group callback to be invoked when the message group is complete. */
  default void addMessageGroupCallback(MessageGroupCallback callback) {
    throw new UnsupportedOperationException();
  }

  /** Closes the connection. */
  default void close() {
    throw new UnsupportedOperationException();
  }
}
