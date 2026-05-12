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

#!/bin/bash
set -ex

if [ -z "${INPUT_ZIP_FILE}" ]; then
  echo "Error: INPUT_ZIP_FILE environment variable is not set."
  exit 1
fi

UNSIGNED_DIR="${KOKORO_ARTIFACTS_DIR}/unsigned"
SIGNED_DIR="${KOKORO_ARTIFACTS_DIR}/signed"
mkdir -p "${UNSIGNED_DIR}" "${SIGNED_DIR}"

find .

mv "${KOKORO_GFILE_DIR}/${INPUT_ZIP_FILE}" "${UNSIGNED_DIR}/"

# Sign using the tool.
export PYTHONPATH=$PYTHONPATH:/escalated_sign

python3 /signing-tools/signer.py \
  --loglevel=debug \
  --output_dir="${SIGNED_DIR}" \
  "${UNSIGNED_DIR}"

OUTPUT_ZIP_NAME=$(basename "${INPUT_ZIP_FILE}")
OUTPUT_ZIP="${KOKORO_ARTIFACTS_DIR}/${OUTPUT_ZIP_NAME}"

cd "${SIGNED_DIR}"
zip -rq "${OUTPUT_ZIP}" .
