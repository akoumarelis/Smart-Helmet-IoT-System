This project is an implementation of a smart flash ecosystem for bicycles, which the cyclist can handle with his helmet when they will light up and the bike will take various measurements. The system consists of two main parts, the helmet, and the bicycle saddle. The cyclist, depending on the movements he makes with the helmet, lights up the corresponding flash or alarm. The bicycle saddle receives commands via ESP-NOW from the helmet about when each flash will light up and sends back data from a pressure sensor located on the saddle that understands when each route starts.The system aims to offer the cyclist automated functions and to be able to see with his Smartphone various metrics that he receives during the route without the need for any action. In other words, it is an IoT ecosystem with sensors placed on the bike and the helmet, which collect data and send it via Wi-Fi to the Server, which is analyzed and visualized through the platform on the cyclist’s mobile phone.

The bike helmet consists of two flashes (left and right), a middle flash that activates when the cyclist slows down, and night lights that are activated only when it is a night for the cyclist to see. The helmet supports according to the above six functions:

    Right flash (2 red LEDs) that turns on or off when the cyclist shakes his helmet twice to the right.
    Left flash (2 red LEDs) that turns on or off when the cyclist shakes his helmet twice to the left.
    Middle flash (1 red led) that activates when the cyclist slows down and deactivates when accelerating.
    Start recording the route that turns on or off when cyclist shakes his helmet down once respectively.
    All flashes are activated when the cyclist shakes his helmet twice up, which means the alarms are on and are deactivated when he shakes his head twice up again.
    Night lights (2 white LEDs) that are activated only at night (detected via Light-dependent resistor) for the cyclist to see in front of him.
The bike saddle consists of two flashes (left and right), a middle flash that activates when the cyclist slows down, and night lights that are activated only when it is a night for the cyclist to see. All functions are the same as the bicycle helmet, only activated when the helmet command is sent via ESP-NOW. The saddle also has a pressure sensor that detects when a route starts but also when the cyclist pedals too hard. The only information that the saddle sends to the helmet is about this sensor.

The bicycle helmet consists of:

    One ESP32-WROOM-32
    A photoresistor or a light-dependent resistor (LDR)
    An MPU 6050 Motion Sensor module: 3-axis gyroscope, 3-axis accelerometer.
    Five red led 3mm (Right (2) – Left (2) flash, brake light (1))
    Two white led 3mm (Light for the night)
    Eight resistors 220 Ohm
    
All the other components mentioned above for the bicycle helmet were placed on the beard-boards.. Below are the pins used in the ESP32 for each component separately:

    Light-dependent resistor (LDR) on pin VN
    Left flash (two red led 3mm) on pins D13-D12
    Right flash (two red led 3mm) on pins D27-D26
    Brake (a red 3mm led) on pins D25
    Light (two white 3mm led) on pins D33-D232
    MPU 6050 Motion Sensor module on pins D21 (SDA) -D22 (SCL)
 
The bicycle Saddle consists of:

    One ESP32-WROOM-32
    Five red led 3mm (Right (2) – Left (2) flash, brake light (1))
    Two white led 3mm (Light for the night)
    Eight resistors 220 Ohm
    One custom pressure sensor

All the other components mentioned above for the bicycle helmet were placed on the beard-boards.. Below are the pins used in the ESP32 for each component separately:

    Left flash (two red led 3mm) on pins D13-D12
    Right flash (two red led 3mm) on pins D27-D26
    Brake (a red 3mm led) on pins D25
    Light (two white 3mm led) on pins D33-D32
    Custom pressure sensor on pin D35    
    
    
![image](https://user-images.githubusercontent.com/57871380/152786860-8597dfb3-fd2b-4a81-8dc1-95a1e897df6b.png)


In the Server part, several different technologies were used for data exchange and visualization. In the structure, we have a Docker in which three programs run on it.After the docker installed run in command line the commands:

    docker run -p 3000:3000 --name grafana
    docker run -p 8086:8086 -v influxdb:/var/lib/influxdb --name infuxdb influxdb:1.8.5
    docker run -it -p 1880:1880 -p 1883:1883 -v node_red_data:/data --name mynodered nodered/node-red
    
![image](https://user-images.githubusercontent.com/57871380/152786355-97147327-fc6d-4111-b730-7b065b2af4a1.png)

The Node-Red receives the data and stores it in the database created within InfluxDB. 

![image](https://user-images.githubusercontent.com/57871380/152786459-0ee55e9f-d304-462b-bc13-ce53eae0b4ec.png)

At the end of the third program, Grafana collects the data from the database and displays it. 

![image](https://user-images.githubusercontent.com/57871380/152786505-ef3fc518-922b-4d37-8b61-f8ccfa1e26fe.png)
![image](https://user-images.githubusercontent.com/57871380/152786510-3d9d2876-9e9b-4ff3-ad06-010ba5e329da.png)
![image](https://user-images.githubusercontent.com/57871380/152786516-1845c6be-b2d6-4772-9bbf-b5bc7ce91633.png)
![image](https://user-images.githubusercontent.com/57871380/152786545-91bb16ef-2ff3-400f-b8ed-68b318713ee9.png)
![image](https://user-images.githubusercontent.com/57871380/152786560-a02c6a8b-4eb1-4621-8c0d-a1eaf2ad1ed8.png)


Personal info:

    Developer: Anastasios Koumarelis (akoumarelis@gmail.com)
    University: Hellenic Mediteranian University, Department of Electrical and Computer Engineering, Msc in Informatics Engineering
    Course: Internet of Things: Technologies and Applications 2020-2021
