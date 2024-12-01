# ResCube
A FreeRTOS project focused on survival.

ResCube is an embedded electronics project designed around survival, built entirely upon Espressif's IoT Development Framework (ESP-IDF), and utilizing several FreeRTOS features such as thread synchronization, thread-safe operations (using mutexes), and multi-threading.

## Features

ResCube is equipped with several features that can assist in various survival scenarios:

1. **EPIRB Emulation**  
   An Emergency Position Indication Rescue Beacon (EPIRB) alerts emergency services by transmitting crucial data, like NMEA coordinates, using radio communication. ResCube emulates this by using a LoRa SX1278 module for basic transmission at various frequencies, combined with a Neo-6M GPS module to parse and send coordinates through the LoRa module.

2. **Stopwatch**  
   A simple stopwatch is implemented and displayed on the OLED display for basic time orientation.

3. **Morse Code Flasher**  
   A Morse code flasher is included, which flashes and buzzes the universal SOS signal (`...---...`) for communication with emergency services (both aerial and marine).

4. **Rangefinders**  
   Two ultrasonic sensors (HC-SR04 and JSN-SR04T) are used for rangefinding operations. These can help measure the depth of falls, water bodies, or even a cave's drop using the JSN-SR04T's waterproof probe.

5. **Proximity/Audio Sensors**
   Swappable sensors that can be used to detect perimeter/entrance intrusion using audio-visual alerts.

## Libraries Used

ResCube makes use of the following libraries:

- [SSD1306 I2C OLED Display](https://github.com/nopnop2002/esp-idf-ssd1306)  
  Library for controlling the SSD1306 OLED display.

- [LoRa SX1278 Module](https://github.com/nopnop2002/esp-idf-sx127x)  
  Library for handling the LoRa Ra-02 (SX1278) module.

---

This project utilizes the power of FreeRTOS and ESP-IDF to create a robust survival tool, combining multiple features that can assist in emergency scenarios.


Outdoors_1-
![1721929019747](https://github.com/user-attachments/assets/1d6650ac-6498-42d7-895d-8d551b665642)
Outdoors_2-
![IMG_20240801_122347](https://github.com/user-attachments/assets/0db684e5-3030-40ab-92da-23f038ac1a56)
Outdoors_3-
![WhatsApp Image 2024-12-01 at 5 15 54 PM](https://github.com/user-attachments/assets/57319153-c62b-4cca-93d0-a5574d73dbf4)
Schematics-
![circuit](https://github.com/user-attachments/assets/536f412e-e37d-413c-9ee5-3ef3d0e422a9)






