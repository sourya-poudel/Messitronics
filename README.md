![image](https://github.com/user-attachments/assets/0b0d9f89-cd07-459f-a930-c893191ce07d)

This WiFi remote control car runs with ESP8266 Module also known as NodeMCU. This module is more Powerful than our traditional Arduino UNO Board. It has more memory and also more processing power. The most important thing is that It has Built-in WiFi Module. This small-sized NodeMCU comes with MicroUSB port and also can be operated with 7-12V. You can also program it with Arduino IDE.

What is WiFi Car?
This is a simple car made up of simple components. This WiFi controlled robot runs with the wifi signal. Let me explain in detail.

First, when we connect the whole circuit with the Power Supply, then the NodeMCU creates a server with the given SSID and the Password. Now we have to connect with the Hotspot and have to open the same IP. (i.e. 192.168.4.1) we can give the signal to the Browser address bar but it does not look professional.

ESP8266 Smart Car:
Suppose You want to give the Robot ESP RC to go to the forward command. Then you have to go to the browser address bar and have to type 192.168.4.1/F (We must have to declare F for forwarding in the code.) for WiFi car Using NodeMCU. In the same way, we have to declare all the preferred directions in the esp8266 wifi control code. Now, One by one typing all the IPs is not good in my opinion. So, Instead of typing every time in the address bar, Here we will use an Android App and from the app, we will give the signals to the microcontroller. With the Instructions, the microcontroller gives data to the motor Driver through the RC ESP connection.

ESP8266 Remote Control Car Schematics:
Here is the schematics/ Circuit Diagram of the ESP8266 WiFi Remote Control Car. For the motor driver, I used L298N. This is a high power motor driver capable of running 5V to 35V DC Motor at a maximum of 25W.
![image](https://github.com/user-attachments/assets/06c10010-1358-48c1-ab2a-96c1ba5dd4f4)

We will use 3S, 12V Li-Po Battery so the maximum output per channel will be around 2A. So, It is more than enough to drive the 12.

Components for a Smart car ESP8266 Project:
Arduino UNO R3: http://bit.ly/2FZY6dT
TT Gear Motor: https://bit.ly/2Mlb35E
ESP8266: https://bit.ly/2Ngfvms
Wheels: https://bit.ly/2U10QPM
3s Li-Po Battery: https://bit.ly/3fJcpUh
HC-05 Bluetooth Module: http://bit.ly/2ukQvod
18650 Battery Holder: https://bit.ly/2MkPYrW
18650 Battery: https://bit.ly/3eBNH7N
L298N Motor Driver: https://bit.ly/3gB8Ws6
Switch: https://bit.ly/3ds4Xfy
Jumper Wires: https://bit.ly/2MjLy4t
Tools Needed for ESP8266 Car:
TS1000 Soldering Iron: http://bit.ly/2FTWtiN
Soldering Wire: https://bit.ly/37MGdwn
Flux: https://bit.ly/2Yjrczf

ESP8266	L298N Motor Driver
D3	Input 4
D4	Input 3
D5	Enable A
D6	Enable B
D7	Input 2
D8	Input 1

Simplified PCB:
![image](https://github.com/user-attachments/assets/5c7ffb2c-c4b2-4005-9f69-262f04dfd9b0)

Codind:
Install board manager
Install the ESP8266 Board Manager
First Download the code from the link below. Now open Arduino and Go to File>New.
Now a new window will appear. Next, Delete all the existing code and Paste the given code.
In the code, you will find Additional Board Manager URL now copy the URL and do the next step. For Different OS you have a different option.
MAC: Go to Arduino > Preferences
Windows: Fille > Preferences
Now Paste it in the Additional Board Manager URL section and press Ok.
Now go to Tools > Board > Boards Manager
Search for ‘ESP8266‘ and install the latest version.
After the installation Then go to Tools > Board and then select the ESP-12E Module. So, The Board is selected Now.
Next, Select the Right COM Port.
Then compile the Programme First and then Upload to NodeMCU. After a few seconds, the code will be compiled and then Uploaded to NodeMCU Car.
Additional Board Manager URL: http://arduino.esp8266.com/stable/package_esp8266com_index.json

Connect the Car with the battery.
Now Install the Application link below.
Connect with the WiFi. In the general case, you will find the Wifi SSID is ‘Sourya’. You can change the wifi Name by changing the SSID.
Now open the app and you will be able to control the car with the app.
The car also has a speed slider by which you can control the speed of the WiFi car Using NodeMCU.
