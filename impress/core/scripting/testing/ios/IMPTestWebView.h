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

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/scripting/testing/web_view_test_runner.h"

#import <Foundation/Foundation.h>
#import <WebKit/WebKit.h>
#import <XCTest/XCTest.h>

NS_ASSUME_NONNULL_BEGIN

/**
 * A wrapper for an iOS WebView for E2E testing.
 */
@interface IMPTestWebView : NSObject <WKScriptMessageHandler>

/**
 * Initializes TestWebView.
 *
 * @param runner Native test runner.
 */
- (instancetype)initWithRunner:(imp::scripting::WebViewTestRunner *)testRunner
    NS_DESIGNATED_INITIALIZER;

- (instancetype)initWithFrame:(CGRect)frame
                configuration:(WKWebViewConfiguration *)configuration NS_UNAVAILABLE;

- (nullable instancetype)initWithCoder:(NSCoder *)coder NS_UNAVAILABLE;

- (instancetype)init NS_UNAVAILABLE;

/**
 * Runs JavaScript tests in the WebView.
 *
 * @param script containing JS tests to run.
 * @return True if JavaScript test suite was successfully completed.
 */
- (bool)runAllTests:(NSString *)script;
/**
 * Formats JS into a function to be executed in the WebView and then evaluates
 * it.
 *
 * @param url of embedded test model.
 */
- (void)evaluateJavaScript:(NSString *)message;

/**
 * The WKWebView object.
 */
@property(nonatomic, readonly) WKWebView *wkWebView;

@end

NS_ASSUME_NONNULL_END
