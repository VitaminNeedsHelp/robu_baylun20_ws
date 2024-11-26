#Schreibe ein Programm mit Systemargumenten. 
#Beim start des Programms soll eine zahl übergeben werden.
#diese dezimale Zahl (0-7) wird als binäre Zahl auf den RGB-LEDs dargestellt.
#wenn als argument "blink" übergeben wird, sollen die LEDs mit der dazugehörigen zahl in abwechselnder farbe blinken.

import signal
import sys
import time
from rpi_ws281x import ws, Adafruit_NeoPixel, Color
import argparse

LED_COUNT = 3 # Anzahl der LEDs
LED_PIN = 21 # GPIO-Pin (Pin 40) (PWM und PCM)
LED_FREQ_HZ = 800000 # Frequenz der LEDs Kommunikationsfrequenz = 800kHz = 1,25µs
LED_DMA = 10 # DMA-Kanal (Direct Memory Access)
LED_BRIGHTNESS = 255 # Helligkeit (0-255) 0 = aus, 255 = max
LED_INVERT = False # Invertierung des Signals (True oder False) True = Invertiert
LED_CHANNEL = 0 # Channel 0 oder 1 (0 = PWM, 1 = PCM)
LED_STRIP = ws.WS2811_STRIP_GRB # Art der LEDs (RGB, GRB, RGBW)

leds = Adafruit_NeoPixel(LED_COUNT,
                            LED_PIN,
                            LED_FREQ_HZ,
                            LED_DMA,
                            LED_INVERT,
                            LED_BRIGHTNESS,
                            LED_CHANNEL,
                            LED_STRIP)

leds.begin()

colors = [Color(255, 0, 0), Color(0, 255, 0), Color(0, 0, 255)]#, Color(0, 255, 255), Color(0, 0, 255), Color(255, 0, 255)]

def turn_off_leds(signal, frame):
    print("Du hast Strg+C gedrückt. Die LEDs werden ausgeschalten.")
    for i in range(LED_COUNT):
        leds.setPixelColor(i, Color(0, 0, 0))
    leds.show()
    sys.exit(0)

# das zweite Argument ist die Funktion, die aufgerufen wird, wenn Strg+C gedrückt wird
#diese Funktion wird als Callback-Funktion oder handler bezeichnet

signal.signal(signal.SIGINT, turn_off_leds)

parser = argparse.ArgumentParser(description="LEDs ansteuern")
parser.add_argument("zahl", type=int, help="Zahl zwischen 0 und 7")
parser.add_argument("--blink", action="store_true", help="Zahl soll blinken")

args = parser.parse_args()

def bin_to_leds(zahl):
    if zahl < 0 or zahl > 7:
        print("Zahl muss zwischen 0 und 7 liegen")
        sys.exit(1)
    for i in range(LED_COUNT):
        if zahl & 1 << i:
            leds.setPixelColor(i, Color(255, 255, 255))
        else:
            leds.setPixelColor(i, Color(0, 0, 0))
    leds.show()

def blink_leds(zahl):
    #blinken in verschiedenen Farben
    for i in range(10):
        for j in range(LED_COUNT):
            if zahl & 1 << j:
                leds.setPixelColor(j, colors[i%3])
            else:
                leds.setPixelColor(j, Color(0, 0, 0))
        leds.show()
        time.sleep(0.5)
        for j in range(LED_COUNT):
            leds.setPixelColor(j, Color(0, 0, 0))
        leds.show()
        time.sleep(0.5)

if args.blink:
    blink_leds(args.zahl)
else:
    bin_to_leds(args.zahl)