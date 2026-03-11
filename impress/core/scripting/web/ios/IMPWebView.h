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

#import <Foundation/Foundation.h>
#import <WebKit/WebKit.h>

namespace imp {
namespace scripting {
class ScriptMessageHandler;
}  // namespace scripting
class Context;
namespace scripting {
class WebView;
}  // namespace scripting
}  // namespace imp

NS_ASSUME_NONNULL_BEGIN
/**
 * A wrapper for a WebView that communicates messages to/from JS using a bridge.
 */
@interface IMPWebView : NSObject <WKNavigationDelegate, WKScriptMessageHandler>

/**
 * Construct an IMPWebView.
 *
 * @param webView The native abstraction of the webview.
 * @param scriptMessageHandler The script message handler to pass incoming messages from JS.
 * @param context The Imp context for the application.
 * @param frame The size of the webview frame.
 * @param url The url to load.
 * @param injectionScript The ImpWeb JavaScript counterpart to inject.
 */
- (instancetype)initWithWebView:(imp::scripting::WebView *)webView
           scriptMessageHandler:(imp::scripting::ScriptMessageHandler &)scriptMessageHandler
                        context:(const imp::Context &)context
                          frame:(CGRect)frame
                            url:(NSString *)url
                injectionScript:(NSString *)injectionScript NS_DESIGNATED_INITIALIZER;

/**
 * Attaches an external WebView to an IMPWebView.
 *
 * @param webView The native abstraction of the webview.
 * @param scriptMessageHandler The script message handler to pass incoming messages from JS.
 * @param context The Imp context for the application.
 * @param injectionScript The ImpWeb JavaScript counterpart to inject.
 * @param externalWebView The external WebView for ImpWebView to attach to.
 */
- (instancetype)initWithWebView:(imp::scripting::WebView *)webView
           scriptMessageHandler:(imp::scripting::ScriptMessageHandler &)scriptMessageHandler
                injectionScript:(NSString *)injectionScript
                externalWebView:(WKWebView *)externalWebView NS_DESIGNATED_INITIALIZER;

/**
 * Injects the last saved script into the WebView (either set in the
 * constructor or last updated from the injectScript overload below).
 */
- (void)injectScript;

/** Updates the saved script in the WebView and then injects it. */
- (void)injectScript:(NSString *)injectionScript;

/**
 * Informs the IMPWebView that the owning IosWebView is being destructed, and
 * that it should prepare to be dealloc-ed.
 */
- (void)onWebViewDestruction;

- (instancetype)initWithFrame:(CGRect)frame
                configuration:(WKWebViewConfiguration *)configuration NS_UNAVAILABLE;

- (nullable instancetype)initWithCoder:(NSCoder *)coder NS_UNAVAILABLE;

- (instancetype)init NS_UNAVAILABLE;

@end

NS_ASSUME_NONNULL_END
