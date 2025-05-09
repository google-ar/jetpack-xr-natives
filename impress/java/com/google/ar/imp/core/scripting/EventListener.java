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

import com.google.protobuf.Any;
import com.google.protobuf.MessageLite;
import java.util.function.Consumer;

/** A helper class to store event listener registrations. */
public final class EventListener<T extends MessageLite> {
  private EventListener(Class<T> eventType, Consumer<T> listener) {
    this.eventType = eventType;
    this.listener = listener;
  }

  /** Creates a new event listener. */
  public static <T extends MessageLite> EventListener<T> create(
      Class<T> eventType, Consumer<T> listener) {
    return new EventListener<T>(eventType, listener);
  }

  private final Class<T> eventType;
  private final Consumer<T> listener;

  /** Handles an event of the expected type. */
  public void onEvent(Any event) {
    this.listener.accept(MessageUtils.getContent(eventType, event));
  }
}
