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

echo_and_run cd "/home/foxy/major_project_ws/src/rtabmap_ros/rtabmap_python"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/foxy/major_project_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/foxy/major_project_ws/install/lib/python3/dist-packages:/home/foxy/major_project_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/foxy/major_project_ws/build" \
    "/usr/bin/python3" \
    "/home/foxy/major_project_ws/src/rtabmap_ros/rtabmap_python/setup.py" \
    egg_info --egg-base /home/foxy/major_project_ws/build/rtabmap_ros/rtabmap_python \
    build --build-base "/home/foxy/major_project_ws/build/rtabmap_ros/rtabmap_python" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/foxy/major_project_ws/install" --install-scripts="/home/foxy/major_project_ws/install/bin"
