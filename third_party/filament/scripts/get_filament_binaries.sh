#!/bin/bash
# run this script from root of a google3 client:
# third_party/filament/scripts/get_filament_binaries.sh v1.3.2 20181213 ~/dev/scratch
# 1st arg: filament tag
# 2nd arg: date stamp for filament assets
# 3rd arg: a directory outside of google3 that this script can use for
#    downloading and extracting archive files
echo ""
echo "------------------------------------------------------"
TAGNAME=$1
DATEID=$2
SCRATCH=$3
WORK=$SCRATCH$TAGNAME"/"
mkdir -p $WORK && rm -rf $WORK/*

# android
wget -O $WORK"android.aar" "https://github.com/google/filament/releases/download/"$TAGNAME"/filament-"$DATEID"-android.aar"

# Indicates the platform of the artifacts we are dowloading. The second field is the file we need from each artifact.
PLATFORM_FILES=("linux:filament/bin/matc" "mac:filament/bin/matc" "windows:bin/matc.exe")

for platform_file in ${PLATFORM_FILES[@]}; do
  PLATFORM=$(echo ${platform_file} | awk -F: '{print $1}')
  FILE_PATH=$(echo ${platform_file} | awk -F: '{print $2}')

  wget  -O ${WORK}${PLATFORM}.tgz https://github.com/google/filament/releases/download/${TAGNAME}/filament-${DATEID}-${PLATFORM}.tgz
  mkdir -p ${WORK}${PLATFORM}/
  tar -xvzf ${WORK}${PLATFORM}.tgz -C ${WORK}${PLATFORM}/ ${FILE_PATH}
done

G3_SCENEFORM_PATH=third_party/arcore/java/com/google/ar/sceneform/

# Copies a downloaded artifact to a third_party location. The string is of the format "local_path_to_artifact:g3_sceneform_destination"
ARTIFACTS=("android.aar:filament_android/filament_android_internal.aar" \
  "linux/filament/bin/matc:sdk/binaries/linux/matc" \
  "mac/filament/bin/matc:sdk/binaries/mac/matc" \
  "windows/bin/matc.exe:sdk/binaries/windows/matc.exe")

for obj in ${ARTIFACTS[@]}; do
  DOWNLOAD=${WORK}$(echo ${obj} | awk -F: '{print $1}')
  OUTPUT_PATH=${G3_SCENEFORM_PATH}$(echo ${obj} | awk -F: '{print $2}')
  g4 edit ${OUTPUT_PATH} && cp ${DOWNLOAD} ${OUTPUT_PATH} && echo "Copied ${DOWNLOAD} to ${OUTPUT_PATH}"
done

echo "Done."
