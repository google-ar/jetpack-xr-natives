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

import androidx.annotation.Nullable;
import com.google.android.filament.proguard.UsedByNative;
import com.google.ar.imp.core.NodeHandleMessageOuterClass.NodeHandleMessage;
import com.google.ar.imp.core.scripting.Bridge.MessageToNative;
import com.google.ar.imp.core.scripting.Bridge.MessageToScript;
import com.google.ar.imp.core.scripting.Events.EventListenerAddRequest;
import com.google.ar.imp.core.scripting.Events.EventListenerAddResponse;
import com.google.ar.imp.core.scripting.Events.EventListenerMessage;
import com.google.ar.imp.core.scripting.Events.EventListenerRemoveRequest;
import com.google.common.util.concurrent.ListenableFuture;
import com.google.common.util.concurrent.SettableFuture;
import com.google.protobuf.Any;
import com.google.protobuf.MessageLite;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

/** Handles MessageToNative and MessageToScript transport between Java and C++. */
public final class ScriptBridge implements ScriptEndpoint, ApiBridge {
  private final long viewHostHandle;

  /**
   * A pending request for an async API call.
   *
   * <p>This class is abstract to allow for both MessageLite (response) and empty (void) responses.
   */
  private abstract static class PendingRequest<T> {
    protected final Class<T> type;
    protected final SettableFuture<T> future;

    PendingRequest(Class<T> type) {
      this.type = type;
      this.future = SettableFuture.create();
    }

    ListenableFuture<T> getFuture() {
      return future;
    }

    void setException(Exception e) {
      future.setException(e);
    }

    /** Resolves the pending request with the given message from native. */
    void resolve(MessageToScript messageToScript) {
      if (!messageToScript.getError().isEmpty()) {
        setException(new ApiException(messageToScript.getError()));
      } else {
        future.set(getResponse(messageToScript));
      }
    }

    abstract T getResponse(MessageToScript messageToScript);
  }

  /** A pending request for an async API call with a MessageLite response. */
  private static class PendingRequestWithResponse<T extends MessageLite> extends PendingRequest<T> {
    PendingRequestWithResponse(Class<T> type) {
      super(type);
    }

    @Override
    T getResponse(MessageToScript messageToScript) {
      return MessageUtils.getContent(type, messageToScript);
    }
  }

  /** A pending request for an async API call with a void response. */
  private static class EmptyPendingRequest extends PendingRequest<Void> {
    EmptyPendingRequest() {
      super(Void.class);
    }

    @Override
    Void getResponse(MessageToScript messageToScript) {
      return null;
    }
  }

  private static class DefaultEventDispatcher implements EventDispatcher {
    @Override
    public <EventT extends MessageLite> void onEvent(EventListener<EventT> listener, Any event) {
      listener.onEvent(event);
    }
  }

  /** set if there is a custom event dispatcher. */
  private EventDispatcher eventDispatcher = new DefaultEventDispatcher();

  /** A map for routing async responses to PendingRequests based on request id. */
  private final Map<Integer, PendingRequest<?>> pendingRequests = new HashMap<>();

  /** A map for routing events to registered EventListeners by proto typeUrl. */
  private final Map<Integer, EventListener<?>> eventListeners = new HashMap<>();

  // Incrementing promise ID (int matches native type).
  private int nextPromiseId = 1;

  /** Creates a new ScriptBridge with the given view host handle for posting messages. */
  public ScriptBridge(long viewHostHandle) {
    this.viewHostHandle = viewHostHandle;
  }

  @Override
  public <RequestT extends MessageLite, ResponseT extends MessageLite> ResponseT sendRequest(
      ApiRequest<RequestT> request, Class<ResponseT> responseType) {
    Any content = MessageUtils.toAny(request.requestTypeUrl(), request.request());
    MessageToNative messageToNative =
        MessageToNative.newBuilder()
            // TODO: don't set id if it's a sync request.
            .setMessageId(nextPromiseId++)
            .setContent(content)
            .build();
    MessageToScript messageToScript =
        sendRequestInternal(messageToNative, request.args(), request.out());
    if (messageToScript != null && !messageToScript.getError().isEmpty()) {
      throw new ApiException(messageToScript.getError());
    }
    if (messageToScript == null) {
      throw new ApiException(
          String.format(
              "Synchronous API call did not return a value. Request type: %s",
              request.requestTypeUrl()));
    }
    if (responseType == null) {
      return null;
    }
    return MessageUtils.getContent(responseType, messageToScript);
  }

  @Override
  public <RequestT extends MessageLite> void sendRequest(ApiRequest<RequestT> request) {
    sendRequest(request, null);
  }

  @Override
  public <RequestT extends MessageLite, ResponseT extends MessageLite>
      ListenableFuture<ResponseT> sendRequestAsync(
          ApiRequest<RequestT> request, Class<ResponseT> responseType) {
    return sendRequestAsyncInternal(
        request, new PendingRequestWithResponse<ResponseT>(responseType));
  }

  @Override
  public <RequestT extends MessageLite> ListenableFuture<Void> sendRequestAsync(
      ApiRequest<RequestT> request) {
    return sendRequestAsyncInternal(request, new EmptyPendingRequest());
  }

  private <RequestT extends MessageLite, ResponseT>
      ListenableFuture<ResponseT> sendRequestAsyncInternal(
          ApiRequest<RequestT> request, PendingRequest<ResponseT> pendingRequest) {
    SettableFuture<ResponseT> future = SettableFuture.create();
    // TODO: This is a bit magical that we know nextPromiseId is post-incremented
    // inside of sendRequest. Refactor to only send promise ids when the request is expected to be
    // async and do that increment in this function for clarity.
    int pendingRequestId = nextPromiseId;
    pendingRequests.put(pendingRequestId, pendingRequest);
    try {
      sendRequest(request);
    } catch (ApiException ex) {
      // An async request can fail immediately, in which case we should set the exception on the
      // future and discard the pending request since we don't expect a message with that ID to be
      // sent in the future.
      pendingRequest.setException(ex);
      pendingRequests.remove(pendingRequestId);
    }
    return pendingRequest.getFuture();
  }

  @Override
  public void setEventDispatcher(EventDispatcher customEventDispatcher) {
    eventDispatcher = customEventDispatcher;
  }

  @Override
  public void removeEventDispatcher() {
    eventDispatcher = new DefaultEventDispatcher();
  }

  @Override
  public <EventT extends MessageLite> EventListenerHandle addEventListener(
      String eventTypeUrl,
      Class<EventT> eventType,
      NodeHandleMessage target,
      Consumer<EventT> listener) {
    EventListenerAddRequest.Builder eventListenerAddRequestBuilder =
        EventListenerAddRequest.newBuilder();
    if (target != null) {
      eventListenerAddRequestBuilder.setTarget(target);
    }
    eventListenerAddRequestBuilder.setEventTypeUrl(
        MessageUtils.getFullyQualifiedProtoTypeUrl(eventTypeUrl));

    EventListenerAddRequest eventListenerAddRequest = eventListenerAddRequestBuilder.build();

    EventListenerAddResponse response =
        sendRequest(
            "imp.scripting.EventListenerAddRequest",
            eventListenerAddRequest,
            EventListenerAddResponse.class);

    Integer id = response.getListenerId();
    EventListenerHandle handle = new EventListenerHandle(this, eventTypeUrl, id);
    eventListeners.put(id, EventListener.create(eventType, listener));
    return handle;
  }

  @Override
  public void removeEventListener(String eventTypeUrl, int id) {
    EventListenerRemoveRequest eventListenerRemoveRequest =
        EventListenerRemoveRequest.newBuilder().setListenerId(id).build();
    sendRequest("imp.scripting.EventListenerRemoveRequest", eventListenerRemoveRequest);
    eventListeners.remove(id);
  }

  /** Sends the given MessageToNative to native, parses and returns the response. */
  @Nullable
  private MessageToScript sendRequestInternal(
      MessageToNative messageToNative, List<Object> args, List<Object> out) {
    byte[] byteArray = messageToNative.toByteArray();
    if (byteArray.length == 0) {
      return null;
    }
    return MessageUtils.parseMessageToScript(nPostMessage(viewHostHandle, byteArray, args, out));
  }

  /** Posts a MessageToScript from native to the Java scripting interface. */
  @Override
  @UsedByNative("web_view.cc")
  public void postMessage(byte[] message) {
    MessageToScript messageToScript = MessageUtils.parseMessageToScript(message);
    Integer id = messageToScript.getMessageId();
    if (id != 0) {
      PendingRequest<?> pendingRequest = pendingRequests.get(id);
      if (pendingRequest != null) {
        pendingRequests.remove(id);
        pendingRequest.resolve(messageToScript);
      }
    } else {
      // If this is not a pending request, it must be an "event", i.e. a message type that comes
      // from
      // C++ without a corresponding API request.
      Any content = messageToScript.getContent();
      if (content
          .getTypeUrl()
          .equals(
              MessageUtils.getFullyQualifiedProtoTypeUrl("imp.scripting.EventListenerMessage"))) {
        EventListenerMessage eventListenerMessage =
            MessageUtils.getContent(EventListenerMessage.class, content);
        EventListener<?> listener = eventListeners.get(eventListenerMessage.getListenerId());
        if (listener != null) {
          eventDispatcher.onEvent(listener, eventListenerMessage.getEvent());
        }
      } else {
        // This is some other message that is not currently handled.
      }
    }
  }

  // LINT.IfChange(scripting)
  private static native byte[] nPostMessage(
      long viewHostHandle, byte[] request, List<Object> args, List<Object> out);
  // LINT.ThenChange(
  //
  // //depot/google3/third_party/impress/core/scripting/web/android/jni/scripting_bridge_jni.cc:scripting
  // )
}
