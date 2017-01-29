// ATtinyX5 fuses L/H/E 0x62 / 0xD6 / 0xFF (1MHz internal clock, 1.8V BOD)
// set fuses:
// avrdude -c stk500v2 -P usb -p t45 -U lfuse:w:0x62:m -U hfuse:w:0xd6:m -U efuse:w:0xff:m -v
//
// ATtiny84A fuses L/H/E 0x62 / 0xD6 / 0xFF (1MHz internal clock, 1.8V BOD)
// avrdude -c stk500v2 -P usb -p t84a -U lfuse:w:0x62:m -U hfuse:w:0xd6:m -U efuse:w:0xff:m -v

#include <avr/sleep.h>
#include <Button.h>            //http://github.com/JChristensen/Button

// pin assignments
const uint8_t led[] = {0, 1};  //led pin numbers

const uint8_t
    REG_EN(2),                 //regulator enable
    BTN_UP(3),
    BTN_DN(4);

// other constants
const bool
    PULLUP(true),
    INVERT(true);

const uint32_t
    DEBOUNCE_MS(25),
    LONG_PRESS(1000),
    SLEEP(30000);

const uint8_t
    BODS(7),                   //BOD Sleep bit in MCUCR
    BODSE(2);                  //BOD Sleep enable bit in MCUCR

Button btnUp(BTN_UP, PULLUP, INVERT, DEBOUNCE_MS);
Button btnDn(BTN_DN, PULLUP, INVERT, DEBOUNCE_MS);
uint8_t br[] = {1, 1};         //led brightness
uint8_t l = 0;                 //index for leds and brightness arrays
unsigned long ms;              //current time from millis()
unsigned long msLast;          //last time a button was pressed

void setup(void)
{
    pinMode(REG_EN, OUTPUT);
    digitalWrite(REG_EN, HIGH);
    delay(5);

    for (uint8_t i=0; i<sizeof(led); i++)
    {
        pinMode(led[i], OUTPUT);
        analogWrite(led[i], 0);
    }
    analogWrite(led[l], br[l]);
}

void loop(void)
{
    ms = millis();
    btnUp.read();
    btnDn.read();

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
        goToSleep();
    }
    else if (ms - msLast >= SLEEP)
    {
        ledsOff();
        goToSleep();
    }
}

void goToSleep(void)
{
    byte adcsra, mcucr1, mcucr2;

    digitalWrite(REG_EN, LOW);
    delay(5);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    PCMSK |= _BV(BTN_UP) | _BV(BTN_DN);
    GIMSK |= _BV(PCIE);                         //enable pin change interrupts
    adcsra = ADCSRA;                            //save ADCSRA
    ADCSRA &= ~_BV(ADEN);                       //disable ADC
    cli();                                      //stop interrupts to ensure the BOD timed sequence executes as required
    mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);    //turn off the brown-out detector
    mcucr2 = mcucr1 & ~_BV(BODSE);              //if the MCU does not have BOD disable capability,
    MCUCR = mcucr1;                             // this code has no effect
    MCUCR = mcucr2;
    sei();                                      //ensure interrupts enabled so we can wake up again
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
    PCMSK = 0;
}

//turn off the LEDs
void ledsOff(void)
{
    for (uint8_t i=0; i<sizeof(led); i++)
    {
        digitalWrite(led[i], LOW);
    }
}

