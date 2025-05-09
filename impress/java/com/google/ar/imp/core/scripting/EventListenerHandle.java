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

package com.google.ar.imp.core.scripting;

/**
 * A handle to an event listener registration on the ApiBridge. Call release() to remove the event
 * listener registration and no longer receive events.
 */
public class EventListenerHandle {
  private final ApiBridge bridge;
  private final String eventTypeUrl;
  private final int id;

  public EventListenerHandle(ApiBridge bridge, String eventTypeUrl, int id) {
    this.bridge = bridge;
    this.eventTypeUrl = eventTypeUrl;
    this.id = id;
  }

  /** Stops the listener from receiving events. Discard this object after calling. */
  public void release() {
    bridge.removeEventListener(eventTypeUrl, id);
  }
}
