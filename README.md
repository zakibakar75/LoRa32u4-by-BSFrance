# BSFrance-LoRa32u4 II
Source code for BSFrance LoRa32u4 II

The board is good. You can buy it at AliExpress.  Quite cheap also.  I found that the performance of the LoRa chip is quite good.  
For the LMIC library, i use my LMIC_AS923 library.  You can find it in my github also.

Note that my board is version 1.3. So, the pin configuration for the LoRa chip is like this :

/* Pin mapping */
const lmic_pinmap lmic_pins = {
        .nss = 8,
        .rxtx = LMIC_UNUSED_PIN,
        .rst = 4,
        .dio = {7, 5, LMIC_UNUSED_PIN},
};

Don't forget, in your Arduino's IDE Preferences-?Additional Boards Manager URL :  https://adafruit.github.io/arduino-board-index/package_adafruit_index.json

And in the Board Manager section, please choose "Adafruit Feather 32u4"

Compile and flash as usual. 
