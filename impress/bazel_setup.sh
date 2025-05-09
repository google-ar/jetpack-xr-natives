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
# This script will setup install and build anything necessary for impress on bazel.
# It also adds entries for ANDROID_HOME and ANDROID_NDK_HOME to your ~/.bashrc.
#
# Usage: $ source ./setup.sh

if [[ -z "{ANDROID_HOME}" || -z "${ANDROID_NDK_HOME}" || "${ANDROID_NDK_HOME}" != "${ANDROID_HOME}/ndk/26.3.11579264" ]]; then
PATH="/google/data/ro/projects/java-platform/linux-amd64/jdk-17-latest/bin:$PATH"
wget https://dl.google.com/android/repository/commandlinetools-linux-11076708_latest.zip
yes | unzip commandlinetools-linux-11076708_latest.zip
yes | cmdline-tools/bin/sdkmanager --sdk_root=$HOME/Android/Sdk --install "platforms;android-34" "build-tools;34.0.0" "ndk;26.3.11579264"
export_android_home="export \"ANDROID_HOME=\$HOME/Android/Sdk\""
export_android_ndk_home="export \"ANDROID_NDK_HOME=\$ANDROID_HOME/ndk/26.3.11579264\""
eval ${export_android_home}
eval ${export_android_ndk_home}
echo "${export_android_home}" >> ~/.bashrc
echo "${export_android_ndk_home}" >> ~/.bashrc
fi

# Install dependencies for building for desktop platform using SDL.
install_required=false
declare -a required_packages=("clang" "lld-14" "lld" "libudev-dev" "libasound2-dev" "libxcursor-dev" "libx11-dev" "libxinerama-dev"
                        "libxrandr-dev" "libcurl4-openssl-dev" "libegl1-mesa-dev" "mesa-common-dev" "bazel-8.0.0")

function is_installed() {
  dpkg --verify "$1" 2>/dev/null
}

for val in ${required_packages[@]}; do
  if ! is_installed $val; then
    echo "Package not found: "
    echo $val
    install_required=true
    break
  fi
done

if $install_required; then
  echo "Installation of missing apt packages required..."
  sudo apt update
  for val in ${required_packages[@]}; do
    sudo apt -y install $val
  done
  echo "Done with apt installs."
fi
