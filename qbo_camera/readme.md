### dependance :
	sudo apt install ros-humble-usb-cam
	

### creer une udev : 

	* prendre les infos de la camera : udevadm info --name=/dev/videoX --query=all
	* remplacer les valeurs lues dans la commande ci dessous

	sudo vi /etc/udev/rules.d/99-qbo-camera.rules
		SUBSYSTEM=="video4linux", ATTRS{idVendor}=="15aa", ATTRS{idProduct}=="1555", SYMLINK+="qbo_eye_camera"

	puis : 
		sudo udevadm control --reload-rules
		sudo udevadm trigger

launch :
	ros2 launch qbo_camera invert_image_launch.py
	ros2 run rqt_image_view rqt_image_view # pour tester le topic


### charge processeurs :

	1.5 % --> usb_cam_node_eye
	7.9 % --> qbo_invert_image
	----------------------
	9.4 %  =  charge CPU supp.