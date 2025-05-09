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

#import "core/scripting/testing/ios/IMPTestWebView.h"

#include <string>

/* Defines block to invoke when evaluateJavaScript completes or fails. */
typedef void (^EvaluateJavaScriptCompletionHandler)(_Nullable id, NSError *_Nullable error);

static NSString *const kJavaScriptFormat = @"(() => {%@})()";
static NSString *const kOnConsoleLogHandlerName = @"onConsoleMessageLog";
static NSString *const kOnConsoleWarnHandlerName = @"onConsoleMessageWarn";
static NSString *const kOnConsoleErrorHandlerName = @"onConsoleMessageError";
static NSString *const kOnConsoleMessageScript =
    @"  console.log = (msg) => {"
    @"    window.webkit.messageHandlers.onConsoleMessageLog.postMessage(msg);"
    @"  };"
    @"  console.warn = (msg) => {"
    @"    window.webkit.messageHandlers.onConsoleMessageWarn.postMessage(msg);"
    @"  };"
    @"  console.error = (msg) => {"
    @"    window.webkit.messageHandlers.onConsoleMessageError.postMessage(msg);"
    @"  };";

@interface IMPTestWebView ()

/**
 * Native test runner.
 */
@property(nonatomic, readonly) imp::scripting::WebViewTestRunner *testRunner;

/**
 * Expectation for JS test suite completion.
 */
@property(nonatomic, nullable) XCTestExpectation *completionExpectation;

@end

@implementation IMPTestWebView

- (instancetype)initWithRunner:(imp::scripting::WebViewTestRunner *)testRunner {
  self = [super init];
  if (self) {
    _testRunner = testRunner;

    // Init WKWebView and attach to view.
    CGRect frame = CGRectMake(0, 0, 1, 1);
    _wkWebView = [[WKWebView alloc] initWithFrame:frame
                                    configuration:[[WKWebViewConfiguration alloc] init]];
    UIWindow *mainWindow = [UIApplication sharedApplication].keyWindow;
    UIViewController *viewController = mainWindow.rootViewController;
    viewController.view = _wkWebView;
    [viewController loadViewIfNeeded];

    // Reroute console logs to native.
    WKUserContentController *contentController = _wkWebView.configuration.userContentController;
    [contentController addScriptMessageHandler:self name:kOnConsoleLogHandlerName];
    [contentController addScriptMessageHandler:self name:kOnConsoleWarnHandlerName];
    [contentController addScriptMessageHandler:self name:kOnConsoleErrorHandlerName];
    [self evaluateJavaScript:kOnConsoleMessageScript];
  }
  return self;
}

- (bool)runAllTests:(NSString *)script {
  self.completionExpectation = [[XCTestExpectation alloc] initWithDescription:@"testingComplete"];
  [self evaluateJavaScript:script];
  XCTWaiterResult result = [XCTWaiter waitForExpectations:@[ self.completionExpectation ]
                                                  timeout:1
                                             enforceOrder:NO];
  while (result == XCTWaiterResultTimedOut) {
    self.completionExpectation = [[XCTestExpectation alloc] initWithDescription:@"testingComplete"];
    self.testRunner->DrainAllExecutors();
    result = [XCTWaiter waitForExpectations:@[ self.completionExpectation ]
                                    timeout:1
                               enforceOrder:NO];
  }
  return result == XCTWaiterResultCompleted;
}

- (void)evaluateJavaScript:(NSString *)message {
  NSMutableString *script = [[NSMutableString alloc] init];
  [script appendFormat:kJavaScriptFormat, message];
  EvaluateJavaScriptCompletionHandler callback = ^(NSString *result, NSError *error) {
    if (error) {
      NSLog(@"Error evaluating JavaScript: %@", error.localizedDescription);
    }
  };
  [self.wkWebView evaluateJavaScript:script completionHandler:callback];
}

#pragma mark - WKScriptMessageHandler Methods

- (void)userContentController:(WKUserContentController *)userContentController
      didReceiveScriptMessage:(WKScriptMessage *)message {
  std::string level = "";
  if ([message.name isEqualToString:kOnConsoleLogHandlerName]) {
    level = "LOG";
  } else if ([message.name isEqualToString:kOnConsoleWarnHandlerName]) {
    level = "WARNING";
  } else if ([message.name isEqualToString:kOnConsoleErrorHandlerName]) {
    level = "ERROR";
  }

  if (![message.body isKindOfClass:[NSString class]]) {
    return;
  }

  std::string msg([message.body UTF8String],
                  [message.body lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
  self.testRunner->OnConsoleMessage(level, msg);
  if ([message.body hasPrefix:@"Testing complete"]) {
    [self.completionExpectation fulfill];
  } else if ([message.body hasPrefix:@"drain_all_executors"]) {
    self.testRunner->DrainAllExecutors();
  }
}

@end
