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

#import "core/input/ios/ios_keyboard_controller.h"
#import "third_party/absl/status/status.h"
#import "third_party/absl/time/time.h"
#import "core/async/executor.h"
#import "core/common/context.h"
#import "core/common/platform_helpers.h"
#import "core/input/input_manager.h"
#import "core/input/key_codes.h"
#import "core/input/keyboard_event.h"
#import "core/ncsb/dispatcher/dispatcher.h"
#import "core/ncsb/dispatcher/event.h"
#import "core/view/base_view.h"
#import "core/view/view_events.h"

#include <CoreText/CoreText.h>
#import <UIKit/UIKit.h>
#include <memory>
#include <string>
#include "core/common/log.h"

@interface KeyInputView : UITextView <UIKeyInput>

@property imp::BaseView *view;

- (KeyInputView *)init:(imp::BaseView *)view;

@end

@implementation KeyInputView

- (KeyInputView *)init:(imp::BaseView *)view {
  self = [super init];
  if (self) {
    _view = view;
  }
  return self;
}

- (BOOL)hasText {
  return YES;
}

- (void)insertText:(NSString *)text {
  const char *c_text = text.UTF8String;
  self.view->GetInputManager().ProcessTextInput(c_text);
}

- (void)deleteBackward {
  // Push BACKSPACE key down.
  uint8_t action = static_cast<uint8_t>(imp::KeyboardEventType::kOnDown);
  imp::Key key = imp::Key(imp::VirtualKeyCode::VK_BACKSPACE, imp::KeyModifier::NONE);
  absl::Duration elapsed_time = absl::Milliseconds(0.1);
  auto status = self.view->GetInputManager().ProcessKeyboardInput(action, key, elapsed_time);
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Unable to process keyboard input: " << status;
    return;
  }

  // Schedule a key up events at the end of the frame.
  self.view->GetDispatcher().Connect(
      [self, key, elapsed_time](const imp::ViewPostFrameEvent &ev) {
        uint8_t action = static_cast<uint8_t>(imp::KeyboardEventType::kOnUp);
        auto res = self.view->GetInputManager().ProcessKeyboardInput(action, key, elapsed_time);
        if (!res.ok()) {
          IMP_LOG(imp::ERROR) << "Unable to process keyboard input: " << res;
        }
        ev.Disconnect();
      },
      self.view);
}

- (BOOL)canBecomeFirstResponder {
  return YES;
}

- (BOOL)canResignFirstResponder {
  return YES;
}

@end

namespace imp {

class iOSKeyboardController::Impl {
 public:
  Impl(const Context &context, BaseView *view) {
    input_view_ = [[KeyInputView alloc] init:view];
    owning_view_ = (__bridge UIView *)context.GetOwningUIView();
  }

  __weak UIView *owning_view_;
  KeyInputView *input_view_;
};

iOSKeyboardController::iOSKeyboardController(const Context &context, BaseView *view) {
  impl_ = std::make_unique<Impl>(context, view);
}

iOSKeyboardController::~iOSKeyboardController() {}

void iOSKeyboardController::OpenKeyboard() {
  [impl_->owning_view_ addSubview:(impl_->input_view_)];
  [impl_->input_view_ becomeFirstResponder];
}

void iOSKeyboardController::CloseKeyboard() { [impl_->input_view_ resignFirstResponder]; }

}  // namespace imp

//
