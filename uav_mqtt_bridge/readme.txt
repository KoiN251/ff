ben web-mqtt
tao 1 cai web don gian
mo .html bang web
len hivemq xem tai khoan mat khau


ben ros2_ws2
tao node python pub sub bang mqtt mosquitto len web
ros2 run uav_mqtt_bridge ros2_mqtt_bridge_px4 --ros-args -p mqtt_host:="d9af036f37434598a035dd87cb93f1eb.s1.eu.hivemq.cloud" -p mqtt_port:=8883 -p mqtt_user:="nhutctuav" -p mqtt_pass:="123456Aa" -p mqtt_use_tls:=True
nhut@nhut:~$ sudo nano /etc/mosquitto/mosquitto.conf
[sudo] password for nhut: 
nhut@nhut:~$ sudo systemctl restart mosquitto
nhut@nhut:~$ sudo ss -ltnp | egrep ':1884|:9001'
            
nhut@nhut:~$ 
