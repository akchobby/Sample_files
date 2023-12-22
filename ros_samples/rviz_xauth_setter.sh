# using docker without unsafe xhost 

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist $DISPLAY | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

docker run -it --rm --name gui_test --net=host \
                                    -e DISPLAY=$DISPLAY \
                                    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                                    -e XAUTHORITY=$XAUTH \
                                    -v $XAUTH:$XAUTH \
                                    osrf/ros:noetic-desktop-full 
