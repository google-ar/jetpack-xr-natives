#!/bin/bash
# Copyright 2024 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


#
# `gen_js.sh` generates a compiled Impress JS binary at
# //third_party/impress/javascript/core/scripting/gen:web.js
# to create necessary files for JS injection during local iOS development on a
# macOS.
#

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1; /bin/pwd -P)
WS_TOP=$( echo ${SCRIPT_DIR} | sed -e 's#\(.*\)/google3/.*$#\1#' )
GOOGLE3=${WS_TOP}/google3
[ -d "${GOOGLE3}" ] || (echo "Can't find google3!"; exit 1)

SRC_DIR=${GOOGLE3}
JAVASCRIPT_DIR="third_party/impress/javascript/core/scripting"

set -x
pushd "$GOOGLE3"
rm -f "blaze-bin/$JAVASCRIPT_DIR/scripting"*
blaze --noexoblaze build third_party/impress/javascript/core/scripting
scp "blaze-bin/$JAVASCRIPT_DIR/web.js" "$JAVASCRIPT_DIR/gen/"
TIMBUS="SUBMIT"
echo " /* DO NOT ${TIMBUS}. Please revert changes or ignore this file in your hg_ignore. */" >> "$JAVASCRIPT_DIR/gen/web.js"

popd
