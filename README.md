Use usb tethering on the DJI RC Pro and set static ip for the laptop. Set laptop ip to `192.168.43.76`, and netmask to `255.255.255.0`

build the image:
``` 
docker compose build
```

start the container:
``` 
docker compose up
```

attach shell:
``` 
docker compose exec -it drone-app /bin/bash
```

then run specific mission inside:
``` 
roslaunch behaviortree_ros run_waypoints.launch 
```
-----------
things to notice:
the actually used `launch/` and `config/` folders are outside of the container and mounted when doing `docker compose up`, so that we don't have to rebuild the image each time we change some parameters.

when starting the container, it will automatically start a roscore, a rosbridge_server, a web_video_server, the web app, rviz and Groot according to the `start_project.sh`.