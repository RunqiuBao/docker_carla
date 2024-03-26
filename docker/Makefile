NAME=dockercarla
VERSION=latest

build:
	docker build -t $(NAME):$(VERSION) .

xhost:
	xhost +local:root

start:
	docker run -it \
    	--env="DISPLAY=$(DISPLAY)" \
	-e SDL_VIDEODRIVER=x11 \
	-e XAUTHORITY=$XAUTHORITY \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    	--volume="$(pwd)/../carla-scripts:/root/code/carla-scripts:rw" \
    	--volume="/mnt/data:/root/data:rw" \
    	--name=dockercarla \
	--gpus 'all,"capabilities=graphics,utility,display,video,compute"' \
	-p 2000-2002:2000-2002 dockercarla \
	bash

contener=`docker ps -a -q`
image=`docker images | awk '/^<none>/ { print $$3 }'`

clean:
	@if [ "$(contener)" != "" ] ; then \
        	docker rm $(contener); \
    	fi

stop:
	docker rm -f $(NAME)

exec:
	docker exec -it $(NAME) /bin/bash

logs:
	docker logs $(NAME)
