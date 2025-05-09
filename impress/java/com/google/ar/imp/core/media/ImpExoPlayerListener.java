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

package com.google.ar.imp.core.media;

import com.google.android.filament.proguard.UsedByNative;

/** Listener for ExoPlayer that interfaces with c++ jni side. */
// LINT.IfChange(ImpExoPlayerListener)
@UsedByNative("android_exoplayer_listener.cc")
public class ImpExoPlayerListener {
  // LINT.ThenChange(
  // //depot/google3/third_party/impress/core/media/android/android_exoplayer_listener.h:ImpExoPlayerListener
  // )

  private final long nativeHandle;

  @UsedByNative("android_exoplayer_listener.cc")
  public ImpExoPlayerListener(long nativeHandle) {
    this.nativeHandle = nativeHandle;
  }

  public void onReady() {
    nOnReady(nativeHandle);
  }

  public void onPlaybackComplete() {
    nOnPlaybackComplete(nativeHandle);
  }

  public void onSeekComplete() {
    nOnSeekComplete(nativeHandle);
  }

  public void onBuffering(int bufferingState) {
    nOnBuffering(nativeHandle, bufferingState);
  }

  // LINT.IfChange(ExoPlayerListener)
  private static native void nOnReady(long nativeHandle);

  private static native void nOnPlaybackComplete(long nativeHandle);

  private static native void nOnSeekComplete(long nativeHandle);

  private static native void nOnBuffering(long nativeHandle, int bufferingState);
  // LINT.ThenChange(
  // //depot/google3/third_party/impress/core/media/android/android_exoplayer_listener.cc:ExoPlayerListener
  // )
}
