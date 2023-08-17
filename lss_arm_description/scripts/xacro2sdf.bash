#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `lss_arm_description` package

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/urdf/lss_arm.urdf.xacro"
TMP_URDF_PATH="/tmp/lss_arm_tmp.urdf"
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

SDF_PATH="$(dirname "${SCRIPT_DIR}")/models/lss_arm_${DOF}dof/model.sdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=lss_arm_"${DOF}"dof
    dof:="${DOF}"
    collision:=true
    ros2_control:=true
    ros2_control_plugin:=ign
    gazebo_preserve_fixed_joint:=false
)

# Remove old SDF file
rm "${SDF_PATH}" 2>/dev/null

# Process xacro into URDF, then convert URDF to SDF and edit the SDF to use relative paths for meshes
xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" -o "${TMP_URDF_PATH}" &&
ign sdf -p "${TMP_URDF_PATH}" | sed "s/model:\/\/lss_arm_description\///g" >"${SDF_PATH}" &&
echo "Created new ${SDF_PATH}"

# Remove temporary URDF file
rm "${TMP_URDF_PATH}" 2>/dev/null
