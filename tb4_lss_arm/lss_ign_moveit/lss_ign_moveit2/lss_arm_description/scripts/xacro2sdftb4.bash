#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `lss_arm_description` package

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/urdf/tb4_lss_arm.urdf.xacro"
SDF_PATH="$(dirname "${SCRIPT_DIR}")/lss_arm/tb4_model.sdf"
TMP_URDF_PATH="/tmp/tb4_lss_arm_tmp.urdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=tb4_lss_arm
    tb4:=true
    gripper:=true
    collision_arm:=true
    collision_gripper:=true
    ros2_control:=true
    ros2_control_plugin:=ign
    ros2_control_command_interface:=effort
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
