#!/bin/sh -x

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

cd "/home/bjornl/ros/workspaces/bjorn_ws/src/rosserial/rosserial_python"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/bjornl/ros/workspaces/bjorn_ws/install/lib/python2.7/dist-packages:/home/bjornl/ros/workspaces/bjorn_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/bjornl/ros/workspaces/bjorn_ws/build" \
    "/usr/bin/python" \
    "/home/bjornl/ros/workspaces/bjorn_ws/src/rosserial/rosserial_python/setup.py" \
    build --build-base "/home/bjornl/ros/workspaces/bjorn_ws/build/rosserial/rosserial_python" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/bjornl/ros/workspaces/bjorn_ws/install" --install-scripts="/home/bjornl/ros/workspaces/bjorn_ws/install/bin"
