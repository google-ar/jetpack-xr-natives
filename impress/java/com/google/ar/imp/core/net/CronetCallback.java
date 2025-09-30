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

package com.google.ar.imp.core.net;

import com.google.android.filament.proguard.UsedByNative;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.Channels;
import java.nio.channels.WritableByteChannel;
import org.chromium.net.CronetException;
import org.chromium.net.UrlRequest;
import org.chromium.net.UrlResponseInfo;

/** Callback class for Cronet requests that bridges to native code. */
@UsedByNative("cronet_url_loader.cc")
public class CronetCallback extends UrlRequest.Callback {
  private static final String TAG = "CronetCallback";
  private static final int BYTE_BUFFER_SIZE = 32 * 1024;

  private final long nativePeer;
  private final ByteArrayOutputStream bytesReceived = new ByteArrayOutputStream();
  private final WritableByteChannel receiveChannel = Channels.newChannel(bytesReceived);

  @UsedByNative("cronet_url_loader.cc")
  public CronetCallback(long nativePeer) {
    this.nativePeer = nativePeer;
  }

  @Override
  public void onRedirectReceived(UrlRequest request, UrlResponseInfo info, String newLocationUrl) {
    request.followRedirect();
  }

  @Override
  public void onResponseStarted(UrlRequest request, UrlResponseInfo info) {
    request.read(ByteBuffer.allocateDirect(BYTE_BUFFER_SIZE));
  }

  @Override
  public void onReadCompleted(UrlRequest request, UrlResponseInfo info, ByteBuffer byteBuffer)
      throws IOException {
    byteBuffer.flip();
    try {
      receiveChannel.write(byteBuffer);
    } catch (IOException e) {
      nativeOnFailed(nativePeer, "IOException onReadCompleted: " + e.getMessage());
      request.cancel();
      return;
    }
    byteBuffer.clear();
    request.read(byteBuffer);
  }

  @Override
  public void onSucceeded(UrlRequest request, UrlResponseInfo info) {
    byte[] responseBody = bytesReceived.toByteArray();
    nativeOnSucceeded(nativePeer, responseBody);
    closeChannel();
  }

  @Override
  public void onFailed(UrlRequest request, UrlResponseInfo info, CronetException error) {
    nativeOnFailed(nativePeer, error.getMessage());
    closeChannel();
  }

  @Override
  public void onCanceled(UrlRequest request, UrlResponseInfo info) {
    nativeOnCanceled(nativePeer);
    closeChannel();
  }

  private void closeChannel() {
    try {
      bytesReceived.close();
      receiveChannel.close();
    } catch (IOException e) {
      // Ignore IOException while closing channels.
    }
  }

  private native void nativeOnSucceeded(long nativePeer, byte[] data);

  private native void nativeOnFailed(long nativePeer, String errorMessage);

  private native void nativeOnCanceled(long nativePeer);
}
