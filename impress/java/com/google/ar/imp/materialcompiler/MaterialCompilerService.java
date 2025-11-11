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

import android.app.Service;
import android.content.Intent;
import android.os.IBinder;
import android.os.ParcelFileDescriptor;
import androidx.annotation.VisibleForTesting;
import com.google.ar.imp.materialcompiler.aidl.IMaterialCompilerService;

/** Service that creates an Android process for compiling materials. */
public final class MaterialCompilerService extends Service {
  private static final String TAG = MaterialCompilerService.class.getSimpleName();
  private final MaterialCompilerServiceImpl binder = new MaterialCompilerServiceImpl();
  private long nativeHandle = 0;

   NativeCallHandler nativeCallHandler = new NativeCallHandlerImpl();

  @Override
  public IBinder onBind(Intent intent) {
    return binder;
  }

  @Override
  public boolean onUnbind(Intent intent) {
    close();

    // Do not allow rebind.
    return false;
  }

  private void create(ParcelFileDescriptor fd, String libraryName) {
    if (libraryName != null && !libraryName.isEmpty()) {
      System.loadLibrary(libraryName);
    }

    if (nativeHandle != 0) {
      nativeCallHandler.close(nativeHandle);
    }

    // Detach the file descriptor - native side now owns the fd.
    nativeHandle = nativeCallHandler.create(fd.detachFd());
  }

  private void close() {
    if (nativeHandle != 0) {
      nativeCallHandler.close(nativeHandle);
    }
    nativeHandle = 0;
  }

  private class MaterialCompilerServiceImpl extends IMaterialCompilerService.Stub {
    @Override
    public void create(ParcelFileDescriptor fd, String libraryName) {
      MaterialCompilerService.this.create(fd, libraryName);
    }

    @Override
    public void close() {
      MaterialCompilerService.this.close();
    }
  }

  interface NativeCallHandler {
    long create(int fd);

    void close(long nativeHandle);
  }

  static class NativeCallHandlerImpl implements NativeCallHandler {
    @Override
    public long create(int fd) {
      return nCreate(fd);
    }

    @Override
    public void close(long nativeHandle) {
      nClose(nativeHandle);
    }
  }

  // LINT.IfChange(native_api)
  private static native long nCreate(int fd);

  private static native void nClose(long nativeHandle);
  // LINT.ThenChange(//depot/google3/third_party/impress/core/materials/compiler/material_compiler_service_jni.cc:native_api)
}
