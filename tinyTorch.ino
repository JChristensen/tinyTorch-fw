// tinyTorch - A small LED flashlight powered by an ATtiny84A.
// https://github.com/JChristensen/tinyTorch-fw
// Copyright (C) 2017 by Jack Christensen and licensed under
// GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html
//
// Hardware design available on GitHub, see:
//   https://github.com/JChristensen/tinyTorch
//
// Works with the ATTiny Core,
//   https://github.com/SpenceKonde/ATTinyCore
// (Pin mapping CCW, Clock 1MHz internal, LTO disabled, BOD 1.8V)
//
// Set fuses: E:0xFF, H:0xD6, L:0x62 (same as factory settings, except 1.8V BOD)
// avrdude -p t84 -U lfuse:w:0x62:m -U hfuse:w:0xd6:m -U efuse:w:0xff:m -v

#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <JC_Button.h>          // https://github.com/JChristensen/JC_Button
#include <movingAvg.h>          // https://github.com/JChristensen/movingAvg

// pin assignments
const uint8_t
    UNUSED_PINS[] = {0, 4, 5, 7, 8},
    LED[] = {3, 2},                 // led pin numbers (white, red)
    REG_EN(1),                      // regulator enable
    DEBUG_LED(6),
    BTN_UP(9),
    BTN_DN(10);

const uint8_t DEFAULT_BRIGHTNESS(31);
const int MIN_VCC(3000);            // millivolts

const uint32_t
    DEBOUNCE_MS(50),
    LONG_PRESS(1000),
    NO_POWEROFF(0),
    SHORT_POWEROFF(60000),
    LONG_POWEROFF(300000),
    VCC_READ_INTERVAL(500),
    TORCH_SIG(0xaa55aa55);

// object declarations
Button btnUp(BTN_UP, DEBOUNCE_MS);
Button btnDn(BTN_DN, DEBOUNCE_MS);
movingAvg Vcc(6);

// global variables
uint8_t l(0);                   // index for LED and brightness arrays
uint8_t br[2];                  // led brightness
uint32_t sleepInterval;         // auto power off interval. stay on forever if zero.
uint32_t signature;             // used as a first-time switch to initialize variables in eeprom
uint32_t ms;                    // current time from millis()
uint32_t msLast;                // last time a button was pressed

// copies of global variables persisted in eeprom
EEMEM uint8_t l_ee;
EEMEM uint8_t br_ee[2];
EEMEM uint32_t sleepInterval_ee;
EEMEM uint32_t signature_ee;

void setup()
{
    // pullups on for noise immunity
    for (uint8_t p=0; p<sizeof(UNUSED_PINS)/sizeof(UNUSED_PINS[0]); p++)
    {
        pinMode(UNUSED_PINS[p], INPUT_PULLUP);
    }

    // initialize the LED pins
    for (uint8_t i=0; i<sizeof(LED)/sizeof(LED[0]); i++)
    {
        pinMode(LED[i], OUTPUT);
        digitalWrite(LED[i], LOW);
    }

    btnDn.begin();
    btnUp.begin();
    Vcc.begin();
    pinMode(DEBUG_LED, OUTPUT);
    pinMode(REG_EN, OUTPUT);
    digitalWrite(REG_EN, HIGH);
    delay(10);
    readParams();

    // check to see if the user wants to set an auto power-off interval
    if (btnDn.isPressed() && btnUp.isPressed())
    {
        sleepInterval = NO_POWEROFF;
    }
    else if (btnDn.isPressed())
    {
        sleepInterval = SHORT_POWEROFF;
    }
    else if (btnUp.isPressed())
    {
        sleepInterval = LONG_POWEROFF;
    }

    // wait for button release so as not to affect brightness
    while (btnDn.read() || btnUp.read());

    // blink LEDs then sleep
    analogWrite(LED[0], DEFAULT_BRIGHTNESS);
    analogWrite(LED[1], DEFAULT_BRIGHTNESS);
    delay(250);
    ledsOff(false);
    delay(250);
    analogWrite(LED[0], DEFAULT_BRIGHTNESS);
    analogWrite(LED[1], DEFAULT_BRIGHTNESS);
    delay(250);
    gotoSleep(false);
}

void loop()
{
    static uint32_t lastRead;

    ms = millis();
    btnUp.read();
    btnDn.read();

    // measure Vcc and sleep if battery is low and regulator is not able to maintain regulated voltage.
    if (ms - lastRead >= VCC_READ_INTERVAL)
    {
        lastRead = ms;
        int avgVcc = Vcc.reading(readVcc());
        if (avgVcc < MIN_VCC)
        {
            ledsOff(false);
            // reset the moving average in case the battery recovers before the next wake up
            Vcc.reset();

            // flash LEDs alternately to indicate low battery
            for (uint8_t i=0; i<16; i++)
            {
                analogWrite(LED[0], DEFAULT_BRIGHTNESS);
                digitalWrite(LED[1], LOW);
                delay(125);
                digitalWrite(LED[0], LOW);
                analogWrite(LED[1], DEFAULT_BRIGHTNESS);
                delay(125);
            }
            br[0] = br[1] = 1;          // minimum brightness
            gotoSleep(false);
        }
    }

    if (btnUp.wasReleased())            // brighter
    {
        msLast = ms;
        br[l] = (br[l] << 1) + 1;
        analogWrite(LED[l], br[l]);
    }
    else if (btnDn.wasReleased())       // dimmer
    {
        msLast = ms;
        br[l] = br[l] >> 1;
        if (br[l] == 0) br[l] = 1;
        analogWrite(LED[l], br[l]);
    }
    else if (btnUp.pressedFor(LONG_PRESS))  // switch color
    {
        msLast = ms;
        digitalWrite(LED[l], LOW);
        if (++l >= sizeof(LED)/sizeof(LED[0])) l = 0;
        analogWrite(LED[l], br[l]);
        while (btnUp.read());                 //wait for release
    }
    else if (btnDn.pressedFor(LONG_PRESS))  // turn off
    {
        gotoSleep(true);
    }
    // auto power-off
    else if (sleepInterval > NO_POWEROFF && ms - msLast >= sleepInterval)
    {
        gotoSleep(true);
    }
}

// pin change interrupt from one of the buttons wakes the MCU
ISR(PCINT0_vect)
{
    GIMSK = 0;      // disable interrupts (only need one to wake up)
    PCMSK0 = 0;
}

