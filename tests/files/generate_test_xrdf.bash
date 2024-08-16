#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <xacro_root>"
    exit 1
fi

CENTAURO_RS_PKG_ROOT=$1
CENTAURO_URDF_ROOT=$CENTAURO_RS_PKG_ROOT/centauro_urdf
CENTAURO_SRDF_ROOT=$CENTAURO_RS_PKG_ROOT/centauro_srdf
ADD_VELODYNE="false"
ADD_REALSENSE="false"
ADD_FLOATING_BASE="true"
ADD_LEGS="true"
ADD_UPPER_BODY="true"

# Check if the path exists
if [ -e "$CENTAURO_RS_PKG_ROOT" ]; then
    xacro $CENTAURO_URDF_ROOT/urdf/centauro.urdf.xacro centauro_root:=$CENTAURO_URDF_ROOT \
        use_abs_mesh_paths:=true velodyne:=$ADD_VELODYNE realsense:=$ADD_REALSENSE floating_joint:=$ADD_FLOATING_BASE \
        legs:=$ADD_LEGS upper_body:=$ADD_UPPER_BODY -o ./centauro.urdf
    xacro $CENTAURO_SRDF_ROOT/srdf/centauro.srdf.xacro centauro_root:=$CENTAURO_URDF_ROOT \
        use_abs_mesh_paths:=true velodyne:=$ADD_VELODYNE realsense:=$ADD_REALSENSE floating_joint:=$ADD_FLOATING_BASE \
        legs:=$ADD_LEGS upper_body:=$ADD_UPPER_BODY -o ./centauro.srdf
else
    echo "Path does not exist: $CENTAURO_RS_PKG_ROOT"
    exit 1
fi
