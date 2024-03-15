Build:

`podman build -t ros .`

Run:

Has to be run once per login session: `xhost +SI:localuser:$(id -un)`

`podman run -ti --rm -e DISPLAY -v $XAUTHORITY --net=host --name=ros ros /bin/bash`
