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


# This script correctly splits proto paths to execute protoc/protocol_compiler.
#
# Realtive paths provided to protoc must match what it written in the file 
# and must not include the search path, which must be specified separately.

#Exit if a pipeline
set -e

usage() {
    cat <<EOM
    Usage: $(basename $0) --protocol_compiler=<protoc binary> --input=<input filename> --output=<output filename> --path=<proto search path>... <proto dependencies>...

EOM
    exit 0
}

# Search paths from command line.
PATH_ARRAY=()
# Explicit dependencies from command line.
DEP_ARRAY=()

while (( $# > 0))
do
    case $1 in
        --protocol_compiler=*)
            COMMAND_LINE="./${1#--protocol_compiler=}"
            ;;
        --input=*)
            INPUT=${1#--input=}
            ;;
        --output=*)
            OUTPUT=${1#--output=}
            ;;
        --path=*)
            PATH_ARRAY+=(${1#--path=})
            ;;
        *)
            DEP_ARRAY+=($1)
            ;;
        "") ;;
    esac
    shift

done

# Check for required parameters
CHECK_CMD_LINE=${OUTPUT:?"Error output not set"}
CHECK_CMD_LINE=${INPUT:?"Error input not set"}
CHECK_CMD_LINE=${COMMAND_LINE:?"Error protocol_compiler not set"}


COMMAND_LINE+=" --encode=imp.NodeData "

if [ ${#PATH_ARRAY[@]} -eq 0 ]; then
    usage
    exit "Error: Proto Path not set"
fi

# Append search path to command line
for protopath in ${PATH_ARRAY[@]}
do
    COMMAND_LINE+=" --proto_path=${protopath}"
done

# Append dependencies to command line
for protopath in ${PATH_ARRAY[@]}
do
    for dependency in ${DEP_ARRAY[@]}
    do
        if [[ $dependency = $protopath* ]]; then
            dependency=${dependency#${protopath}/}
            #if it starts with this proto path, remove the prefix.
            COMMAND_LINE+=" ${dependency}"
        fi
    done
done

echo "${COMMAND_LINE}  < ${INPUT} > ${OUTPUT}"
exec ${COMMAND_LINE}  < ${INPUT} > ${OUTPUT}
