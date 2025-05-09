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

import com.google.ar.imp.core.NodeHandleMessageOuterClass.NodeHandleMessage;
import com.google.common.util.concurrent.ListenableFuture;
import com.google.protobuf.MessageLite;
import java.util.function.Consumer;

/** The interface for a bridge to native code for sending requests and receiving responses. */
public interface ApiBridge {
  /** Constructs a new Node wrapper given a nodeHandleMessage returned from native. */
  default Node wrapNode(NodeHandleMessage nodeHandleMessage) {
    return new Node(this, nodeHandleMessage);
  }

  /** Sends a synchronous request to native and returns a response of the given type. */
  <RequestT extends MessageLite, ResponseT extends MessageLite> ResponseT sendRequest(
      ApiRequest<RequestT> request, Class<ResponseT> responseType);

  /** Sends a synchronous request to native and returns a response of the given type. */
  default <RequestT extends MessageLite, ResponseT extends MessageLite> ResponseT sendRequest(
      String requestTypeUrl, RequestT request, Class<ResponseT> responseType) {
    return sendRequest(new ApiRequest<RequestT>(requestTypeUrl, request), responseType);
  }

  /** Sends a synchronous request to native and expects no response. */
  <RequestT extends MessageLite> void sendRequest(ApiRequest<RequestT> request);

  /** Sends a synchronous request to native and expects no response. */
  default <RequestT extends MessageLite> void sendRequest(String requestTypeUrl, RequestT request) {
    sendRequest(new ApiRequest<RequestT>(requestTypeUrl, request));
  }

  /** Sends an asynchronous request to native, returns a ListenableFuture of the response type. */
  <RequestT extends MessageLite, ResponseT extends MessageLite>
      ListenableFuture<ResponseT> sendRequestAsync(
          ApiRequest<RequestT> request, Class<ResponseT> responseType);

  /** Sends an asynchronous request to native, returns a ListenableFuture of the response type. */
  default <RequestT extends MessageLite, ResponseT extends MessageLite>
      ListenableFuture<ResponseT> sendRequestAsync(
          String requestTypeUrl, RequestT request, Class<ResponseT> responseType) {
    return sendRequestAsync(new ApiRequest<RequestT>(requestTypeUrl, request), responseType);
  }

  /** Sends an asynchronous request to native and expects no response. */
  <RequestT extends MessageLite> ListenableFuture<Void> sendRequestAsync(
      ApiRequest<RequestT> request);

  /** Sends a synchronous request to native and expects no response. */
  default <RequestT extends MessageLite> ListenableFuture<Void> sendRequestAsync(
      String requestTypeUrl, RequestT request) {
    return sendRequestAsync(new ApiRequest<RequestT>(requestTypeUrl, request));
  }

  /**
   * Adds an event listener to the given event proto type and optional Node target. If no Node
   * target is specified, listener will receive global messages of that type. Returns an
   * EventListenerHandle - to remove this listener, call handle.release().
   */
  <EventT extends MessageLite> EventListenerHandle addEventListener(
      String eventTypeUrl,
      Class<EventT> eventType,
      NodeHandleMessage target,
      Consumer<EventT> listener);

  /** Removes a previously-registered event listener. Clients should call handle.release(). */
  void removeEventListener(String eventTypeUrl, int id);

  /** A custom EventDispatcher might be set to intercept and/or modify events. */
  void setEventDispatcher(EventDispatcher customEventDispatcher);

  /** Removes any custom event dispatcher. */
  void removeEventDispatcher();
}
