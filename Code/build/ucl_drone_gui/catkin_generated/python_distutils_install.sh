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

echo_and_run cd "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone_gui"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/install/lib/python2.7/dist-packages:/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/build" \
    "/usr/bin/python" \
    "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/ucl_drone_gui/setup.py" \
    build --build-base "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/build/ucl_drone_gui" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/install" --install-scripts="/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/install/bin"
