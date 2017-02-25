#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/michael/Documents/VSCode_Workspace/CSE180/CSE180_Workspace/src/gazebo_ros_pkgs/gazebo_plugins"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/michael/Documents/VSCode_Workspace/CSE180/CSE180_Workspace/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/michael/Documents/VSCode_Workspace/CSE180/CSE180_Workspace/install/lib/python2.7/dist-packages:/home/michael/Documents/VSCode_Workspace/CSE180/CSE180_Workspace/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/michael/Documents/VSCode_Workspace/CSE180/CSE180_Workspace/build" \
    "/usr/bin/python" \
    "/home/michael/Documents/VSCode_Workspace/CSE180/CSE180_Workspace/src/gazebo_ros_pkgs/gazebo_plugins/setup.py" \
    build --build-base "/home/michael/Documents/VSCode_Workspace/CSE180/CSE180_Workspace/build/gazebo_ros_pkgs/gazebo_plugins" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/michael/Documents/VSCode_Workspace/CSE180/CSE180_Workspace/install" --install-scripts="/home/michael/Documents/VSCode_Workspace/CSE180/CSE180_Workspace/install/bin"
