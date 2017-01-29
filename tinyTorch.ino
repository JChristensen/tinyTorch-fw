// ATtinyX5 fuses L/H/E 0x62 / 0xD6 / 0xFF (1MHz internal clock, 1.8V BOD)
// set fuses:
// avrdude -c stk500v2 -P usb -p t45 -U lfuse:w:0x62:m -U hfuse:w:0xd6:m -U efuse:w:0xff:m -v
//
// ATtiny84A fuses L/H/E 0x62 / 0xD6 / 0xFF (1MHz internal clock, 1.8V BOD)
// avrdude -c stk500v2 -P usb -p t84a -U lfuse:w:0x62:m -U hfuse:w:0xd6:m -U efuse:w:0xff:m -v

#include <avr/sleep.h>
#include <Button.h>            //http://github.com/JChristensen/Button
#include <movingAvg.h>         //http://github.com/JChristensen/movingAvg

void gotoSleep(bool forever = false);

// pin assignments
const uint8_t led[] = {2, 3};  //led pin numbers

const uint8_t
    REG_EN(6),                //regulator enable
    BTN_UP(9),
    BTN_DN(8);

// other constants
const bool
    PULLUP(true),
    INVERT(true);

const uint32_t
    DEBOUNCE_MS(25),
    LONG_PRESS(1000),
    SHORT_SLEEP(60000),
    LONG_SLEEP(300000);

const int MIN_VCC(3500);         //millivolts

//const uint8_t                  //not needed with newer AVR Libc
//    BODS(7),                   //BOD Sleep bit in MCUCR
//    BODSE(2);                  //BOD Sleep enable bit in MCUCR

Button btnUp(BTN_UP, PULLUP, INVERT, DEBOUNCE_MS);
Button btnDn(BTN_DN, PULLUP, INVERT, DEBOUNCE_MS);
movingAvg Vcc;

uint8_t br[] = {1, 1};         //led brightness
uint8_t l = 0;                 //index for leds and brightness arrays
unsigned long ms;              //current time from millis()
unsigned long msLast;          //last time a button was pressed
unsigned long sleepInterval;   //auto power off interval. stay on forever if zero.

void setup(void)
{
    pinMode(REG_EN, OUTPUT);
    digitalWrite(REG_EN, HIGH);
    delay(5);
    
    //check to see if the user wants to set an auto power-off interval
    btnDn.read();
    btnUp.read();
    if (btnDn.isPressed())
    {
        sleepInterval = SHORT_SLEEP;
    }
    else if (btnUp.isPressed())
    {
        sleepInterval = LONG_SLEEP;
    }

    for (uint8_t i=0; i<sizeof(led); i++)
    {
        pinMode(led[i], OUTPUT);
        analogWrite(led[i], 0);
    }
    analogWrite(led[l], br[l]);
    
    //wait for button release so as not to adjust brightness
    btnUp.read();
    btnDn.read();
    while (btnDn.isPressed() || btnUp.isPressed())
    {
        btnUp.read();
        btnDn.read();
    }
}

void loop(void)
{
    const uint32_t VCC_READ_INTERVAL = 1000;
    static uint32_t lastRead;
    
    ms = millis();
    btnUp.read();
    btnDn.read();

    //Vcc measurement, if battery is low and regulator not able to maintain regulated voltage,
    //turn off LEDs, shut down the regulator, and go into power-down sleep mode forever or until a reset whichever comes first.
    if (ms - lastRead >= VCC_READ_INTERVAL)
    {
        lastRead += VCC_READ_INTERVAL;
        int v = readVcc();
        int avgVcc = Vcc.reading(v);
        if (avgVcc < MIN_VCC)
        {
            ledsOff();
            for (uint8_t i=0; i<16; i++)
            {
                digitalWrite(led[0], HIGH);
                digitalWrite(led[1], LOW);
                delay(125);
                digitalWrite(led[0], LOW);
                digitalWrite(led[1], HIGH);
                delay(125);
            }
            ledsOff();
            digitalWrite(REG_EN, LOW);
            gotoSleep(true);
        }
    }

    if (btnUp.wasReleased())
    {
        msLast = ms;
        br[l] = (br[l] << 1) + 1;
        analogWrite(led[l], br[l]);
    }
    else if (btnDn.wasReleased())
    {
        msLast = ms;
        br[l] = br[l] >> 1;
        if (br[l] == 0) br[l] = 1;
        analogWrite(led[l], br[l]);
    }
    else if (btnUp.pressedFor(LONG_PRESS))
    {
        msLast = ms;
        digitalWrite(led[l], LOW);
        if (++l >= sizeof(led)) l = 0;
        analogWrite(led[l], br[l]);
        while (!btnUp.wasReleased()) btnUp.read();
    }
    else if (btnDn.pressedFor(LONG_PRESS))
    {
        ledsOff();
        while (!btnDn.wasReleased()) btnDn.read();
        gotoSleep();
    }
    else if (sleepInterval > 0 && ms - msLast >= sleepInterval)
    {
        ledsOff();
        gotoSleep();
    }
}

void gotoSleep(bool forever)
{
    digitalWrite(REG_EN, LOW);
    delay(5);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
//    PCMSK |= _BV(BTN_UP) | _BV(BTN_DN);
//    GIMSK |= _BV(PCIE);                         //enable pin change interrupts
    if (!forever)
    {
        PCMSK0 |= _BV(PCINT2) | _BV(PCINT1);        //enable pin change interrupts ATtinyX4
        GIMSK |= _BV(PCIE0);                        //enable pin change interrupts ATtinyX4
    }
    uint8_t adcsra = ADCSRA;                    //save ADCSRA
    ADCSRA &= ~_BV(ADEN);                       //disable ADC
    cli();                                      //stop interrupts to ensure the BOD timed sequence executes as required
    uint8_t mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);    //turn off the brown-out detector
    uint8_t mcucr2 = mcucr1 & ~_BV(BODSE);      //if the MCU does not have BOD disable capability,
    MCUCR = mcucr1;                             // this code has no effect
    MCUCR = mcucr2;
    if (!forever) sei();                        //ensure interrupts enabled so we can wake up again
    sleep_cpu();                                //go to sleep
    sleep_disable();                            //wake up here
    ADCSRA = adcsra;                            //restore ADCSRA

    digitalWrite(REG_EN, HIGH);
    delay(5);
    analogWrite(led[l], br[l]);
    ms = millis();
    msLast = ms;

    btnUp.read();
    btnDn.read();
    while (!btnDn.wasReleased() && !btnUp.wasReleased())
    {
        btnUp.read();
        btnDn.read();
    }
}

//pin change interrupt from one of the buttons wakes the MCU
ISR(PCINT0_vect)
{
    GIMSK = 0;                                  //disable interrupts (only need one to wake up)
    PCMSK0 = 0;
}

//turn off the LEDs
void ledsOff(void)
{
    for (uint8_t i=0; i<sizeof(led); i++)
    {
        digitalWrite(led[i], LOW);
    }
}

//read 1.1V reference against Vcc
//from http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
int readVcc(void)
{
    ADMUX = _BV(MUX5) | _BV(MUX0);
    delay(5);                                 //Vref settling time
    ADCSRA |= _BV(ADSC);                      //start conversion
    loop_until_bit_is_clear(ADCSRA, ADSC);    //wait for it to complete
    return 1126400L / ADC;                    //calculate AVcc in mV (1.1 * 1000 * 1024)
}

