// tinyTorch - A small LED flashlight powered by an ATtiny84A.
// by Jack Christensen Jan-2017
//
// Hardware design and firmware are available on GitHub, see:
// This firmware available at http://goo.gl/xpB4pj
//
// Set fuses: E:0xFF, H:0xD6, L:0x62 (same as factory settings, except 1.8V BOD)
// avrdude -p t84 -U lfuse:w:0x62:m -U hfuse:w:0xd6:m -U efuse:w:0xff:m -v
//
// "tinyTorch" by Jack Christensen is licensed under CC BY-SA 4.0,
//  http://creativecommons.org/licenses/by-sa/4.0/

#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <Button.h>            //http://github.com/JChristensen/Button
#include <movingAvg.h>         //http://github.com/JChristensen/movingAvg

// pin assignments
const uint8_t
UNUSED_PINS[] = {0, 4, 5, 7, 8},
LED[] = {3, 2},            //led pin numbers (white, red)
REG_EN(1),                 //regulator enable
DEBUG_LED(6),
BTN_UP(9),
BTN_DN(10);

// other constants
const bool
PULLUP(true),
INVERT(true);

const uint8_t DEFAULT_BRIGHTNESS(31);

//const uint8_t                  //not needed with newer AVR Libc
//    BODS(7),                   //BOD Sleep bit in MCUCR
//    BODSE(2);                  //BOD Sleep enable bit in MCUCR

const int
MIN_VCC(3000),               //millivolts
NOMINAL_VCC(3300);

const uint32_t
DEBOUNCE_MS(25),
LONG_PRESS(1000),
NO_POWEROFF(0),
SHORT_POWEROFF(60000),
LONG_POWEROFF(300000),
VCC_READ_INTERVAL(500),
TORCH_SIG(0xaa55aa55);

// object declarations
Button btnUp(BTN_UP, PULLUP, INVERT, DEBOUNCE_MS);
Button btnDn(BTN_DN, PULLUP, INVERT, DEBOUNCE_MS);
movingAvg Vcc;

// global variables
uint8_t l(0);                  //index for LED and brightness arrays
uint8_t br[2];                 //led brightness
uint32_t sleepInterval;        //auto power off interval. stay on forever if zero.
uint32_t signature;            //used as a first-time switch to initialize variables in eeprom
uint32_t ms;                   //current time from millis()
uint32_t msLast;               //last time a button was pressed

// copies of global variables persisted in eeprom
EEMEM uint8_t l_ee;
EEMEM uint8_t br_ee[2];
EEMEM uint32_t sleepInterval_ee;
EEMEM uint32_t signature_ee;

void setup(void)
{
    //pullups on for noise immunity
    for (uint8_t p=0; p<sizeof(UNUSED_PINS); p++)
    {
        pinMode(UNUSED_PINS[p], INPUT_PULLUP);
    }
    pinMode(DEBUG_LED, OUTPUT);
    pinMode(REG_EN, OUTPUT);
    digitalWrite(REG_EN, HIGH);
    delay(10);
    readParams();

    //check to see if the user wants to set an auto power-off interval
    btnDn.read();
    btnUp.read();
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

    //wait for button release so as not to affect brightness
    btnUp.read();
    btnDn.read();
    while (btnDn.isPressed() || btnUp.isPressed())
    {
        btnUp.read();
        btnDn.read();
    }

    //initialize the LED pins
    for (uint8_t i=0; i<sizeof(LED); i++)
    {
        pinMode(LED[i], OUTPUT);
        analogWrite(LED[i], 0);
    }
    analogWrite(LED[l], br[l]);    //initial setting
}

void loop(void)
{
    static uint32_t lastRead;

    ms = millis();
    btnUp.read();
    btnDn.read();

    //measure Vcc and sleep if battery is low and regulator is not able to maintain regulated voltage.
    if (ms - lastRead >= VCC_READ_INTERVAL)
    {
        lastRead = ms;
        int avgVcc = Vcc.reading(readVcc());
        if (avgVcc < MIN_VCC)
        {
            ledsOff(false);

            //artificially load the Vcc moving average in case the battery
            //recovers before the next wake up. else there would probably be
            //an immediate low battery indication again due to the moving average.
            for (uint8_t i=0; i<6; i++)
            {
                Vcc.reading(NOMINAL_VCC);
            }

            //flash LEDs alternately to indicate low battery
            for (uint8_t i=0; i<16; i++)
            {
                analogWrite(LED[0], DEFAULT_BRIGHTNESS);
                digitalWrite(LED[1], LOW);
                delay(125);
                digitalWrite(LED[0], LOW);
                analogWrite(LED[1], DEFAULT_BRIGHTNESS);
                delay(125);
            }
            ledsOff(false);
            br[0] = br[1] = 1;            //minimum brightness
            gotoSleep();
        }
    }

    if (btnUp.wasReleased())              //brighter
    {
        msLast = ms;
        br[l] = (br[l] << 1) + 1;
        analogWrite(LED[l], br[l]);
    }
    else if (btnDn.wasReleased())         //dimmer
    {
        msLast = ms;
        br[l] = br[l] >> 1;
        if (br[l] == 0) br[l] = 1;
        analogWrite(LED[l], br[l]);
    }
    else if (btnUp.pressedFor(LONG_PRESS))    //switch color
    {
        msLast = ms;
        digitalWrite(LED[l], LOW);
        if (++l >= sizeof(LED)) l = 0;
        analogWrite(LED[l], br[l]);
        while (!btnUp.wasReleased()) btnUp.read();
    }
    else if (btnDn.pressedFor(LONG_PRESS))    //turn off
    {
        ledsOff(true);
        while (!btnDn.wasReleased()) btnDn.read();
        gotoSleep();
    }
    //auto power-off
    else if (sleepInterval > NO_POWEROFF && ms - msLast >= sleepInterval)
    {
        ledsOff(true);
        gotoSleep();
    }
}

//pin change interrupt from one of the buttons wakes the MCU
ISR(PCINT0_vect)
{
    GIMSK = 0;                                  //disable interrupts (only need one to wake up)
    PCMSK0 = 0;
}

