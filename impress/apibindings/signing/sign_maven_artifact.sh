# Copyright 2026 Google LLC
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

STAGING_DIR="${KOKORO_ARTIFACTS_DIR}/staging"
mkdir -p "${STAGING_DIR}"

export PYTHONPATH=$PYTHONPATH:/escalated_sign

for ZIP_PATH in "${KOKORO_GFILE_DIR}"/*.zip; do
  ZIP_NAME=$(basename "${ZIP_PATH}")

  WORK_DIR="${KOKORO_ARTIFACTS_DIR}/work_${ZIP_NAME}"
  mkdir -p "${WORK_DIR}"

  unzip "${ZIP_PATH}" -d "${WORK_DIR}"

  find "${WORK_DIR}" -type f \( -name "*.aar" -o -name "*.jar" -o -name "*.pom" \) | while read ARTIFACT_PATH; do
    ARTIFACT_NAME=$(basename "${ARTIFACT_PATH}")
    ARTIFACT_DIR=$(dirname "${ARTIFACT_PATH}")

    echo "Signing ${ARTIFACT_NAME}..."

    cp "${ARTIFACT_PATH}" "${STAGING_DIR}/${ARTIFACT_NAME}"

    /escalated_sign/escalated_sign.py \
      --tool=linux_gpg_sign \
      --job-dir=/escalated_sign_jobs \
      -- \
      --loglevel=debug \
      "${STAGING_DIR}/${ARTIFACT_NAME}"

    if [ -f "${STAGING_DIR}/${ARTIFACT_NAME}.asc" ]; then
        mv "${STAGING_DIR}/${ARTIFACT_NAME}.asc" "${ARTIFACT_DIR}/"
    else
        echo "Error: Signature not created for ${ARTIFACT_NAME}"
        exit 1
    fi

    rm "${STAGING_DIR}/${ARTIFACT_NAME}"
  done

  cd "${WORK_DIR}"
  zip -rq "${KOKORO_ARTIFACTS_DIR}/${ZIP_NAME}" .
  cd -

  rm -rf "${WORK_DIR}"
done
