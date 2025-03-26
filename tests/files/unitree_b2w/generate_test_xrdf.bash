#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <xacro_root>"
    exit 1
fi

UNITREE_B2W_RS_PKG_ROOT=$1
UNITREE_B2W_XRDF_ROOT=$UNITREE_B2W_RS_PKG_ROOT
ADD_FLOATING_BASE="true"

# Check if the path exists
if [ -e "$UNITREE_B2W_RS_PKG_ROOT" ]; then
    xacro $UNITREE_B2W_XRDF_ROOT/xacro/robot.urdf.xacro root:=$UNITREE_B2W_XRDF_ROOT \
        use_abs_mesh_paths:=true floating_joint:=$ADD_FLOATING_BASE \
        -o ./unitree_b2w.urdf
    xacro $UNITREE_B2W_XRDF_ROOT/xacro/robot.srdf.xacro root:=$UNITREE_B2W_XRDF_ROOT \
        use_abs_mesh_paths:=true floating_joint:=$ADD_FLOATING_BASE \
        -o ./unitree_b2w.srdf
else
    echo "Path does not exist: $UNITREE_B2W_RS_PKG_ROOT"
    exit 1
fi
