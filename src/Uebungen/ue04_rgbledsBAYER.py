import signal
import sys
import time
from rpi_ws281x import ws, Adafruit_NeoPixel, Color

#https://github.com/rpi-ws281x/rpi-ws281x-python
#https://cdn.sparkfun.com/datasheets/BreakoutBoards/WS2812B.pdf
#https://pypi.org/project/rpi-ws281x/
#Installieren: sudo pip install rpi_ws281x

LED_COUNT = 3 # Anzahl der LEDs
LED_PIN = 21 # GPIO-Pin (Pin 40) (PWM und PCM)
LED_FREQ_HZ = 800000 # Frequenz der LEDs Kommunikationsfrequenz = 800kHz = 1,25µs
LED_DMA = 10 # DMA-Kanal (Direct Memory Access)
LED_BRIGHTNESS = 255 # Helligkeit (0-255) 0 = aus, 255 = max
LED_INVERT = False # Invertierung des Signals (True oder False) True = Invertiert
LED_CHANNEL = 0 # Channel 0 oder 1 (0 = PWM, 1 = PCM)
LED_STRIP = ws.WS2811_STRIP_GRB # Art der LEDs (RGB, GRB, RGBW)

ampel = Adafruit_NeoPixel(LED_COUNT,
                          LED_PIN,
                          LED_FREQ_HZ,
                          LED_DMA,
                          LED_INVERT,
                          LED_BRIGHTNESS,
                          LED_CHANNEL,
                          LED_STRIP)

ampel.begin()

colors = [Color(255, 0, 0), Color(0, 255, 0), Color(0, 0, 255)]#, Color(0, 255, 255), Color(0, 0, 255), Color(255, 0, 255)]

def turn_off_leds(signal, frame):
    print("Du hast Strg+C gedrückt. Die LEDs werden ausgeschalten.")
    for i in range(LED_COUNT):
        ampel.setPixelColor(i, Color(0, 0, 0))
    ampel.show()
    sys.exit(0)

# das zweite Argument ist die Funktion, die aufgerufen wird, wenn Strg+C gedrückt wird
#diese Funktion wird als Callback-Funktion oder handler bezeichnet
signal.signal(signal.SIGINT, turn_off_leds) 

#while True:
#    for i in range(len(colors)):
#       for j in range(LED_COUNT):
#            ampel.setPixelColor(j, colors[i])
#        ampel.show()
#        time.sleep(1)

#color gradient übergang

while True:
    for i in range(len(colors)):
        for j in range(LED_COUNT):
            ampel.setPixelColor(j, colors[i])
        ampel.show()
        time.sleep(1)
    for i in range(len(colors)-1, 0, -1):
        for j in range(LED_COUNT):
            ampel.setPixelColor(j, colors[i])
        ampel.show()
        time.sleep(1)