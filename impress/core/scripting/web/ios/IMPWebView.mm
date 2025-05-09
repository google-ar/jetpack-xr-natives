// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#import "core/scripting/web/ios/IMPWebView.h"

#import <UIKit/UIKit.h>
#import <WebKit/WebKit.h>

#include "core/common/log.h"
#include "core/common/context.h"
#include "core/scripting/message_helpers.h"
#include "core/scripting/proto/bridge.proto.imp.h"
#include "core/scripting/web/web_view.h"

NS_ASSUME_NONNULL_BEGIN

/* Defines block to invoke when evaluateJavaScript completes or fails. */
typedef void (^EvaluateJavaScriptCompletionHandler)(_Nullable id, NSError *_Nullable error);

// LINT.IfChange
static NSString *const kNativeEntryPoint = @"nativeEntryPoint";
// LINT.ThenChange(//depot/google3/third_party/impress/javascript/core/scripting/api/window.ts)

// LINT.IfChange
static NSString *const kJavaScriptEntryPoint =
    @"window.javaScriptEntryPoint && window.javaScriptEntryPoint.incoming && "
    @"window.javaScriptEntryPoint.incoming.postMessage";
// LINT.ThenChange(
//   //depot/google3/third_party/impress/javascript/core/scripting/api/script_bridge.ts
// )

static NSString *const kWrongInitializerError = @"Wrong initializer - use initWithContext";
static NSString *const kMessageParseError = @"Invalid message received from JS";

static NSString *const kInjectionFormat = @"(()=>{%@})();";
static NSString *const kMessageToScriptFormat = @"%@('%@');";

static int const kTimeoutIntervalSeconds = 100;

@interface IMPWebView ()

/**
 * The actual WebView object.
 */
@property(nonatomic, readonly) WKWebView *wkWebView;

/**
 * The native representation of the webview to use for communicating to/from the Impress engine.
 */
@property(nonatomic, readonly) imp::scripting::WebView *webView;

@end

@implementation IMPWebView {
  NSString *_injectionScript;
}

- (instancetype)initWithWebView:(imp::scripting::WebView *)webView
                        context:(const imp::Context &)context
                          frame:(CGRect)frame
                            url:(NSString *)url
                injectionScript:(NSString *)injectionScript {
  // Values of -1 should be interpreted as "full frame" to match Android behavior.
  CGRect screenBounds = [UIScreen mainScreen].bounds;
  if (frame.size.width < 0) {
    frame.size.width = CGRectGetWidth(screenBounds);
  }
  if (frame.size.height < 0) {
    frame.size.height = CGRectGetHeight(screenBounds);
  }

  self = [super init];
  if (self) {
    _webView = webView;
    _wkWebView = [[WKWebView alloc] initWithFrame:frame
                                    configuration:[[WKWebViewConfiguration alloc] init]];
    _wkWebView.navigationDelegate = self;

    _wkWebView.opaque = NO;
    _wkWebView.backgroundColor = [UIColor clearColor];

    UIView *owningView = (__bridge UIView *)context.GetOwningUIView();
    [owningView addSubview:_wkWebView];
    [owningView bringSubviewToFront:_wkWebView];

    // TODO: Handle invalid or malformed URLs.
    NSURLRequest *request = [NSURLRequest requestWithURL:[NSURL URLWithString:url]
                                             cachePolicy:NSURLRequestReloadIgnoringLocalCacheData
                                         timeoutInterval:kTimeoutIntervalSeconds];

    [self configureWebViewBridge:injectionScript];
    [_wkWebView loadRequest:request];
  }
  return self;
}

- (instancetype)initWithWebView:(imp::scripting::WebView *)webView
                injectionScript:(NSString *)injectionScript
                externalWebView:(WKWebView *)externalWebView {
  self = [super init];
  if (self) {
    _webView = webView;
    _wkWebView = externalWebView;
    [self configureWebViewBridge:injectionScript];
  }
  return self;
}

- (void)postMessage:(NSString *)message {
  if (_webView->GetState() == imp::scripting::WebView::State::kUnavailable) {
    return;
  }
  EvaluateJavaScriptCompletionHandler callback = ^(NSString *result, NSError *error) {
    if (error) {
      IMP_LOG(imp::ERROR) << "Failed to evaluate JS with error: '" << [error.debugDescription UTF8String]
                 << "'";
    }
  };
  [self.wkWebView evaluateJavaScript:[self formatMessage:message] completionHandler:callback];
}

- (void)injectScript {
  EvaluateJavaScriptCompletionHandler callback = ^(NSString *result, NSError *error) {
    if (error) {
      IMP_LOG(imp::ERROR) << "Failed to inject JS with error: '" << [error.debugDescription UTF8String]
                 << "'";
    }
  };
  [self.wkWebView evaluateJavaScript:_injectionScript completionHandler:callback];
}

- (void)injectScript:(NSString *)injectionScript {
  [self updateInjectionScript:injectionScript];
  [self injectScript];
}

- (void)onWebViewDestruction {
  // Remove the handler, or this object will be stuck in a retain cycle.
  [self.wkWebView.configuration.userContentController
      removeScriptMessageHandlerForName:kNativeEntryPoint];
}

#pragma mark - Private Helper Methods

/**
 * Configures bidirectional communication between native and JS by registering for messages sent
 * from JS via addScriptMessageHandler and injecting the ImpWeb JS counterpart into the WKWebView.
 */
- (void)configureWebViewBridge:(NSString *)script {
  // Add handler to receive messages sent from JS.
  [self.wkWebView.configuration.userContentController addScriptMessageHandler:self
                                                                         name:kNativeEntryPoint];
  // Inject ImpWeb JS counterpart into the WebView at document end so that the user script can
  // load and add its 'impwebready' event listener to the window.
  [self updateInjectionScript:script];
}

/** Formats the message to pass to the JS native entry point. */
- (NSString *)formatMessage:(NSString *)message {
  NSMutableString *str = [[NSMutableString alloc] init];
  [str appendFormat:kMessageToScriptFormat, kJavaScriptEntryPoint, message];
  return [str copy];
}

/**
 * Saves script string in injectable format and sets it as the user script to inject whenever the
 * WebView DOM finishes loading.
 */
- (void)updateInjectionScript:(NSString *)script {
  NSMutableString *injectionScript = [[NSMutableString alloc] init];
  [injectionScript appendFormat:kInjectionFormat, script];
  _injectionScript = injectionScript;
  WKUserScript *userScript =
      [[WKUserScript alloc] initWithSource:_injectionScript
                             injectionTime:WKUserScriptInjectionTimeAtDocumentEnd
                          forMainFrameOnly:false];
  [self.wkWebView.configuration.userContentController addUserScript:userScript];
}

#pragma mark - WKScriptMessageHandler Methods

- (void)userContentController:(WKUserContentController *)userContentController
      didReceiveScriptMessage:(WKScriptMessage *)message {
  id body = message.body;
  if (![body isKindOfClass:[NSString class]]) {
    // TODO: Send error to JS.
    IMP_LOG(imp::ERROR) << [kMessageParseError UTF8String];
    return;
  }
  NSData *data = [[NSData alloc] initWithBase64EncodedString:body options:0];
  imp::scripting::MessageToNative messageProto = imp::scripting::MessageToNative();
  if (!ParseFromArray(data.bytes, data.length, &messageProto)) {
    // TODO: Send error to JS.
    IMP_LOG(imp::ERROR) << [kMessageParseError UTF8String];
    return;
  }
  self.webView->HandleMessage(messageProto);
}

#pragma mark - WKWebView.navigationDelegate methods

- (void)webView:(WKWebView *)webView
    didFailProvisionalNavigation:(null_unspecified WKNavigation *)navigation
                       withError:(NSError *)error {
  // TODO: report the error back to JS.
  IMP_LOG(imp::ERROR) << [error.localizedDescription UTF8String];
}

- (void)webView:(WKWebView *)webView
    didFailNavigation:(null_unspecified WKNavigation *)navigation
            withError:(NSError *)error {
  // TODO: report the error back to JS.
  IMP_LOG(imp::ERROR) << [error.localizedDescription UTF8String];
}

- (void)webViewDidClose:(WKWebView *)webView {
  self.webView->OnWebViewDestroyed();
}

@end

NS_ASSUME_NONNULL_END
