#!/usr/bin/env bash

set -eo pipefail

project_dir=/home/tang/tsy/indoor_localization

source /opt/ros/humble/setup.bash
source /home/tang/autoware/install/setup.bash
source "${project_dir}/install/setup.bash"

rviz_config="${project_dir}/install/room_global_localizer/share/room_global_localizer/rviz/room_global_localizer.rviz"

exec /usr/bin/env \
  -u SNAP -u SNAP_LIBRARY_PATH \
  -u GTK_PATH -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE \
  -u GDK_PIXBUF_MODULEDIR -u GDK_PIXBUF_MODULE_FILE -u GIO_MODULE_DIR \
  -u LOCPATH -u XDG_DATA_HOME -u XDG_DATA_DIRS \
  rviz2 -d "${rviz_config}"
