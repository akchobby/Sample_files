# LOOK OUT

Sucessfully able to run firefox gui using compose

# Extra to dos

* COOKIE authentication not need ?
* COOKIE generate using:

```bash
 $ xauth generate $DISPLAY . trusted 
 $ export COOKIE=$(xauth list | head -n 1)

```

* Finding right Xauthority path using:

```bash

ps aux | grep -i xorg
```


Note for rviz2 the issue still persists , reference repo is [rocker](https://github.com/osrf/rocker/blob/main/src/rocker/nvidia_extension.py)

nvidia variables during build included that may have helped were found in [ros documentation](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration)
