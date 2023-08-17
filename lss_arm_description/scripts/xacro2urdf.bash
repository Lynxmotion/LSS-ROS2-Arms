#!/usr/bin/env bash
# This script converts xacro (URDF variant) into URDF for `lss_arm_description` package

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/urdf/lss_arm.urdf.xacro"

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

URDF_PATH="$(dirname "${SCRIPT_DIR}")/urdf/lss_arm_${DOF}dof.urdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=lss_arm_"${DOF}"dof
    dof:="${DOF}"
    collision:=true
    ros2_control:=true
    ros2_control_plugin:=ign
    gazebo_preserve_fixed_joint:=false
)

# Remove old URDF file
rm "${URDF_PATH}" 2>/dev/null

# Process xacro into URDF
xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" -o "${URDF_PATH}" &&
echo "Created new ${URDF_PATH}"
