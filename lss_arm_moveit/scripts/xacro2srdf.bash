#!/usr/bin/env bash
# This script converts xacro (SRDF variant) into SRDF for `lss_arm_description` package

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/srdf/lss_arm.srdf.xacro"

# Default parameters
DOF=4

# Parse arguments
while getopts "d:" arg; do
  case ${arg} in
     d) case ${OPTARG} in
          4|5) DOF="${OPTARG}" ;;
          *) echo "Invalid DOF: ${OPTARG}. DOF can only be 4 or 5." >&2
             exit 1 ;;
        esac ;;
     *) echo "Invalid option: -${OPTARG}" >&2
        exit 1;;
  esac
done

SRDF_PATH="$(dirname "${SCRIPT_DIR}")/srdf/lss_arm_${DOF}dof.srdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=lss_arm
    dof:="${DOF}"
)

# Remove old SRDF file
rm "${SRDF_PATH}" 2>/dev/null

# Process xacro into SRDF
xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" -o "${SRDF_PATH}" &&
echo "Created new ${SRDF_PATH}"
