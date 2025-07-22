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

package com.google.ar.imp.view;

import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import androidx.annotation.Nullable;
import androidx.concurrent.futures.ResolvableFuture;
import com.google.common.util.concurrent.ListenableFuture;
import java.util.concurrent.Callable;
import java.util.concurrent.Executor;

/**
 * Coordinates the Impress frame loop & manages the Impress threading model.
 *
 * <p>The frame loop advances using the Android choreographer. If a frame fails to render (i.e. the
 * render thread is behind or ARCore didn't have a new frame yet), then the frame is retried.
 *
 * <p>The FrameScheduler can be configured to run on either the main thread or a background thread.
 * By default, the main thread is used. This is generally performant since rendering is still
 * internally multi-threaded.
 *
 * <p>When running on a background thread, all calls into Impress must happen on the background
 * thread. This class provides methods for running work on the frame thread.
 */
public abstract class FrameScheduler {

  /** The runtime state of the frame scheduler. */
  public enum State {
    STOPPED,
    RUNNING,
  }

  // TODO: (broken link) - Don't use an enum ((broken link)).
  /** Determines the thread that the frame scheduler will run on - main or background. */
  public enum ThreadMode {
    // Impress will run on the main thread using the choreographer. In most
    // cases, this should be fine for performance since rendering is still done
    // on a background thread. However, long running work using the Impress API
    // could block the main thread. The benefit of this is that calls can be
    // made between Impress and android APIs without needing to hop between
    // threads.
    MAIN_DEFAULT,
    // Impress will run on a background thread using the choreographer.
    // All calls into Impress must happen from the background thread. That means
    // that custom jni calls or calls into Impress scripting should use
    // impApi.getFrameScheduler() to execute the work on the background thread.
    BACKGROUND,
  }

  /** Used as a callback to advance a frame when the frame loop is running. */
  public interface FrameAdvancer {
    /**
     * Advances the frame.
     *
     * <p>Returns -1 if the frame successfully advanced. Otherwise, indicates that the frame was
     * skipped. The returned number provides a hint for how long we should wait to retry the frame
     * in milliseconds.
     */
    long advanceFrame(long frameTimeNanos);
  }

  /**
   * Manages the thread that the frame scheduler is running on and provides a handler for scheduling
   * work on that thread.
   */
  public interface FrameThread {
    /**
     * Returns the handler for the frame thread which can be used to schedule work on the thread.
     */
    Handler getHandler();

    /**
     * Destroys the frame thread. In MAIN_DEFAULT, this doesn't actually do anything. In BACKGROUND
     * mode, this will stop the thread.
     */
    void destroy();
  }

  /**
   * Used when configured to run on the main thread. Provides a handler for posting to the main
   * thread.
   */
  private static class MainFrameThread implements FrameThread {
    private static final Handler MAIN_THREAD_HANDLER = new Handler(Looper.getMainLooper());

    @Override
    public Handler getHandler() {
      return MAIN_THREAD_HANDLER;
    }

    @Override
    public void destroy() {
      // No-op.
    }
  }

  /**
   * Used when configured to run on a background thread. Creates the thread and provides a handler
   * for posting to it.
   */
  private static class BackgroundFrameThread implements FrameThread {
    private final HandlerThread thread;
    private final Handler handler;

    public BackgroundFrameThread(@Nullable String threadName) {
      if (threadName == null || threadName.isEmpty()) {
        threadName = "impressThread";
      }

      thread = new HandlerThread(threadName);
      thread.start();

      handler = new Handler(thread.getLooper());
    }

    @Override
    public Handler getHandler() {
      return handler;
    }

    @Override
    public void destroy() {
      thread.quitSafely();
    }
  }

  /** Abstract factory allows an override of the FrameScheduler */
  public interface Factory {
    public default FrameScheduler create(ThreadMode threadMode) {
      return create(threadMode, null);
    }

    /**
     * Creates a FrameScheduler.
     *
     * @param threadMode The thread mode for the frame scheduler.
     * @param threadName The name of the thread. Only used in background mode.
     */
    public abstract FrameScheduler create(ThreadMode threadMode, @Nullable String threadName);
  }

  protected FrameThread frameThread;

  protected FrameScheduler(ThreadMode threadMode) {
    this(threadMode, null);
  }

  protected FrameScheduler(ThreadMode threadMode, @Nullable String threadName) {
    switch (threadMode) {
      case MAIN_DEFAULT:
        frameThread = new MainFrameThread();
        break;
      case BACKGROUND:
        frameThread = new BackgroundFrameThread(threadName);
        break;
    }
  }

  /**
   * Initiates the frame loop and stops the previous frame loop if one was already running. The
   * FrameAdvancer will be called each frame based on the choreographer tick on the appropriate
   * thread. If the FrameAdvancer skips a frame it will be retried.
   */
  public abstract void startFrameLoop(FrameAdvancer frameAdvancer);

  /** Stops the current frame loop if one is running. */
  public abstract void stopFrameLoop();

  /** Returns true if the current thread is the thread the frame scheduler runs on. */
  public boolean isOnFrameThread() {
    return Looper.myLooper() == getLooper();
  }

  /**
   * Throws an IllegalStateException if the current thread is not the thread the frame scheduler
   * runs on.
   */
  public void checkFrameThread() {
    if (!isOnFrameThread()) {
      throw new IllegalStateException("Not on frame scheduler thread.");
    }
  }

  /**
   * Posts work on the frame thread. If already on the frame thread, then the work is executed
   * immediately.
   */
  public void runOnFrameThread(Runnable runnable) {
    if (isOnFrameThread()) {
      runnable.run();
    } else {
      frameThread.getHandler().post(runnable);
    }
  }

  /**
   * Impress is integrated in the Jetpack XR developer SDK, and can't depend on Guava as per Jetpack
   * policy. As an exception, we are allowed to use the ListenableFuture interface only, but now
   * also use ResolvableFuture which inherits from AbstractFuture just like ListenableFuture.
   * ResolvableFuture is not meant to be part of any public API interface, so we need to return it
   * as a ListenableFuture. The "RestrictTo" warning ensures we don't expose ResolvableFuture to
   * clients, but it is safe for us to suppress it until Impress lives in the Jetpack repo and using
   * "ResolvableFuture" becomes standard practice.
   */
  /**
   * Submits work that produces a result on the frame thread. This is similar to using
   * Futures.submit. The difference is that if already on the frame thread the work will execute
   * immediately.
   */
  @SuppressWarnings("RestrictTo")
  public <T> ListenableFuture<T> submitOnFrameThread(Callable<T> task) {
    ResolvableFuture<T> future = ResolvableFuture.create();

    Runnable runnable =
        () -> {
          try {
            T result = task.call();
            future.set(result);
          } catch (Exception e) {
            future.setException(e);
          }
        };

    if (isOnFrameThread()) {
      runnable.run();
    } else {
      frameThread.getHandler().post(runnable);
    }

    return future;
  }

  /**
   * Returns an Executor that can be used to post callbacks to the frame scheduler's thread. Can be
   * used with ListenableFuture.
   */
  public Executor getExecutor() {
    return frameThread.getHandler()::post;
  }

  /** Returns the looper from the selected FrameThread */
  public Looper getLooper() {
    return frameThread.getHandler().getLooper();
  }

  /** Destroys the frame thread. */
  public void destroy() {
    frameThread.destroy();
  }
}
