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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/hr/aaworkspace/simulate/xtdrone_bags/src/gazebo_ros_pkgs/gazebo_plugins"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hr/aaworkspace/simulate/xtdrone_bags/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hr/aaworkspace/simulate/xtdrone_bags/install/lib/python3/dist-packages:/home/hr/aaworkspace/simulate/xtdrone_bags/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hr/aaworkspace/simulate/xtdrone_bags/build" \
    "/usr/bin/python3" \
    "/home/hr/aaworkspace/simulate/xtdrone_bags/src/gazebo_ros_pkgs/gazebo_plugins/setup.py" \
     \
    build --build-base "/home/hr/aaworkspace/simulate/xtdrone_bags/build/gazebo_ros_pkgs/gazebo_plugins" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/hr/aaworkspace/simulate/xtdrone_bags/install" --install-scripts="/home/hr/aaworkspace/simulate/xtdrone_bags/install/bin"
