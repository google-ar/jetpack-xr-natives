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
import android.os.Looper;
import android.os.Message;
import androidx.annotation.Nullable;

/**
 * Creates a frame schedulers which does not depend on the Choreographer.
 *
 * <p>Advances the frame in a async tight loop. Frame pacing must be implemented in the underlying
 * app or library because the Choreographer is not used. ImpApi's option to run on background thread
 * is not implemented. ImpApi's retry frame mechanism is not supported.
 */
public final class ContinuousFrameScheduler extends FrameScheduler {

  /** Factory method for the ContinuousFrameScheduler */
  public static class Factory implements FrameScheduler.Factory {
    @Override
    public FrameScheduler create(ThreadMode threadMode, String threadName) {
      if (instance == null) {
        instance = new ContinuousFrameScheduler(threadMode, threadName);
      }
      return instance;
    }

    private static ContinuousFrameScheduler instance = null;
  }

  private static final class ContinuousFrameHandler extends Handler {
    final ContinuousFrameScheduler scheduler;

    public ContinuousFrameHandler(ContinuousFrameScheduler scheduler, Looper looper) {
      super(looper);
      this.scheduler = scheduler;
    }

    @Override
    public void handleMessage(Message msg) {
      scheduler.doFrame();
    }
  }

  private final ContinuousFrameHandler handler;
  private State state = State.STOPPED;
  @Nullable private FrameAdvancer frameAdvancer;

  ContinuousFrameScheduler(ThreadMode threadMode) {
    this(threadMode, null);
  }

  ContinuousFrameScheduler(ThreadMode threadMode, String threadName) {
    super(threadMode, threadName);
    handler = new ContinuousFrameHandler(this, getLooper());
    frameAdvancer = null;
    state = State.STOPPED;
  }

  @Override
  public void startFrameLoop(FrameScheduler.FrameAdvancer frameAdvancer) {
    runOnFrameThread(
        () -> {
          synchronized (ContinuousFrameScheduler.this) {
            clearHandler();
            ContinuousFrameScheduler.this.frameAdvancer = frameAdvancer;
            asyncRequestFrame();
          }
        });
  }

  @Override
  public void stopFrameLoop() {
    runOnFrameThread(
        () -> {
          synchronized (ContinuousFrameScheduler.this) {
            clearHandler();
          }
        });
  }

  private void clearHandler() {
    frameAdvancer = null;
    handler.removeCallbacksAndMessages(null);
    state = State.STOPPED;
  }

  private void asyncRequestFrame() {
    checkFrameThread();
    handler.removeCallbacksAndMessages(null);
    Message msg = handler.obtainMessage();
    msg.setAsynchronous(true);
    handler.sendMessage(msg);
    state = State.RUNNING;
  }

  private void doFrame() {
    if (state == State.STOPPED) {
      return;
    }

    checkFrameThread();
    if (frameAdvancer == null) {
      return;
    }

    // TODO: Get Impress FrameTime from OpenXR instead of System.nanoTime.
    // Note: ImpApi's retry frame mechanism is not supported in OpenXr, duplicate frames can not be
    // submitted.
    @SuppressWarnings("unused")
    long retryIsNotUsed = frameAdvancer.advanceFrame(System.nanoTime());
    asyncRequestFrame();
  }

  public synchronized State getState() {
    return state;
  }
}
