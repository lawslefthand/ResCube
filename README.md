# ResCube
A FreeRTOS project based on the aspect of survival.

ResCube is an embedded electronics project that focuses on the aspect of survival while being built entirely upon Espressif's Iot-Devlopment framework (Esp IDF) while using several FreeRTOS features and concepts like Thread synchronisation , thread safe operations (Using Mutexes) and multi-threading. 

It has several features which can help in various aspects of survival situations like -

1) EPIRB Emulation - An emergency position indication rescue beacon is a device which alerts various emergency service providers by sending crucial data like NMEA coordinates to emergency services using radio communication. ResCube emulates that by using a simple Lora SX1278 module for basic transmission at various frequencies and a Neo 6m gps module through which coordinates are parsed and sent through the Lora module.

2) Stopwatch - A simple stopwatch that can be displayed on the OLED display for basic orientation of time.

3) Morse code flasher- A Morse code is flashed and buzzed for emergency services (aerial and marine) for basic communication. Only one Morse code is loaded and that's the universal
SOS (...---...) .

4) Rangefinders - two ultrasonic sensors (Hc-SR04 and JSN-SR04) are used for rangefinding operations which can help gauge the depth of falls , water bodies and even a cave's drop using the jsn-sr04's waterproof probe.

Rescube using the following libraries for its operations -

https://github.com/nopnop2002/esp-idf-ssd1306 -For SSD1306 I2C oled display

https://github.com/nopnop2002/esp-idf-sx127x - For Lora Ra02 Module (SX1278)

Outdoors_1-
![1721929019747](https://github.com/user-attachments/assets/1d6650ac-6498-42d7-895d-8d551b665642)
Outdoors_2-
![IMG_20240801_122347](https://github.com/user-attachments/assets/0db684e5-3030-40ab-92da-23f038ac1a56)
Schematics-
![circuit](https://github.com/user-attachments/assets/536f412e-e37d-413c-9ee5-3ef3d0e422a9)






