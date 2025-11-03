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

import com.google.ar.imp.core.scripting.Bridge.MessageToScript;
import com.google.protobuf.Any;
import com.google.protobuf.ExtensionRegistryLite;
import com.google.protobuf.InvalidProtocolBufferException;
import com.google.protobuf.MessageLite;

/** Utility functions for dealing with protos. */
public final class MessageUtils {

  private static final String GOOGLEAPIS_PROTO_PREFIX = "type.googleapis.com/";

  /**
   * Gets the default instance of the given MessageLite proto.
   *
   * <p>Note: Adapted from java/com/google/protobuf/contrib/MessageUtils.java
   */
  private static <T extends MessageLite> T getDefaultInstance(Class<T> type) {
    try {
      return type.cast(type.getMethod("getDefaultInstance").invoke(null));
    } catch (ReflectiveOperationException | ClassCastException e) {
      throw new IllegalArgumentException(
          String.format(
              "Failed to getDefaultInstance() of type %s. "
                  + "Ensure that your proguard_specs list includes "
                  + "\"//third_party/impress/build_tools/platforms/android:proguard.pgcfg\".",
              type.getName()),
          e);
    }
  }

  public static String getFullyQualifiedProtoTypeUrl(String typeUrl) {
    return GOOGLEAPIS_PROTO_PREFIX + typeUrl;
  }

  /** Converts the given MessageLite proto to an any with the given typeUrl set. */
  public static <T extends MessageLite> Any toAny(String typeUrl, T message) {
    return Any.newBuilder()
        .setTypeUrl(getFullyQualifiedProtoTypeUrl(typeUrl))
        .setValue(message.toByteString())
        .build();
  }

  /** Deserializes the given byte array into a MessageToScript proto. */
  public static MessageToScript parseMessageToScript(byte[] message) {
    try {
      return MessageToScript.parseFrom(message, ExtensionRegistryLite.getEmptyRegistry());
    } catch (InvalidProtocolBufferException e) {
      throw new ApiException(String.format("Invalid proto buffer response: %s", e.getMessage()), e);
    }
  }

  /** Converts the given Any proto into the given MessageLite type. */
  public static <T extends MessageLite> T getContent(Class<T> contentType, Any contentAny) {
    try {
      Object content =
          getDefaultInstance(contentType)
              .getParserForType()
              .parseFrom(contentAny.getValue(), ExtensionRegistryLite.newInstance());

      if (contentType.isInstance(content)) {
        return contentType.cast(content);
      } else {
        throw new ApiException(
            String.format(
                "Content was not of the expected type. Content typeUrl: %s, expected type: %s",
                contentAny.getTypeUrl(), contentType.getSimpleName()));
      }
    } catch (InvalidProtocolBufferException e) {
      throw new ApiException(
          String.format("Invalid content in message received from native: %s", e.getMessage()), e);
    }
  }

  /** Converts the content Any of the given MessageToScript into the given MessageLite type. */
  public static <T extends MessageLite> T getContent(
      Class<T> contentType, MessageToScript message) {
    return getContent(contentType, message.getContent());
  }

  private MessageUtils() {}
}
