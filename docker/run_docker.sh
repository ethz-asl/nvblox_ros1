#!/bin/bash

# Based on the ETH Robotics Summer school docker: 
# https://github.com/ETHZ-RobotX/smb_docker/

# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.

# Default options
DOCKER=nvblox
DOCKERFILE=Dockerfile
NAME=nvblox
BUILD=false
WORKSPACE=/home/$USER/nvblox_ws

help()
{
    echo "Usage: run_docker.sh 
               [ -b | --build ] [ -n | --name <docker name> ]
               [ -h | --help  ] [ -w | --workspace </workspace/path> ]"
    exit 2
}

SHORT=b,n:,w:,h
LONG=build,name:,workspace:,help
OPTS=$(getopt -a -n run_docker --options $SHORT --longoptions $LONG -- "$@")
echo $OPTS

eval set -- "$OPTS"

while :
do
  case "$1" in
    -b | --build )
      BUILD="true"
      shift
      ;;
    -n | --name )
      NAME="$2"
      shift 2
      ;;
    -w | --workspace )
      WORKSPACE="$2"
      shift 2
      ;;
    -h | --help)
      help
      ;;
    --)
      shift;
      break
      ;;
    *)
      echo "Unexpected option: $1"
      help
      ;;
  esac
done


if [ "$BUILD" = true ]; then
    echo "Building docker: $DOCKERFILE as $DOCKER"
    docker build -f $DOCKERFILE -t $DOCKER .
fi

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ -n "$xauth_list" ]; then
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

docker run -it --rm \
    --env="DISPLAY=$DISPLAY" \
    --env="FRANKA_IP=$FRANKA_IP" \
    --volume=$WORKSPACE:/root/nvblox_ws \
    --volume=/home/$USER/data:/root/data \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    --name=$NAME \
    ${DOCKER} \
    bash

echo "Done."
