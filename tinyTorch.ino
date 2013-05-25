//fuses L/H/E 0x62 / 0xD6 / 0xFF
//set fuses:
//avrdude -c stk500v2 -P usb -p t45 -U lfuse:w:0x62:m -U hfuse:w:0xd6:m -U efuse:w:0xff:m -v

#include <avr/sleep.h>
#include <Button.h>            //http://github.com/JChristensen/Button
#define RED_LED 0
#define WHT_LED 1
#define REG_EN 2               //regulator enable
#define BTN_UP 3
#define BTN_DN 4
#define PULLUP true
#define INVERT true
#define DEBOUNCE_MS 25
#define LONG_PRESS 1000
#define SLEEP 10000

Button btnUp(BTN_UP, PULLUP, INVERT, DEBOUNCE_MS);
Button btnDn(BTN_DN, PULLUP, INVERT, DEBOUNCE_MS);
uint8_t led[] = {0, 1};        //led pin numbers
uint8_t br[] = {1, 1};         //led brightness
uint8_t l = 0;                 //index for leds and brightness arrays
unsigned long ms;
unsigned long msLast;

void setup(void)
{
    pinMode(REG_EN, OUTPUT);
    digitalWrite(REG_EN, HIGH);
    delay(5);
    
    for (uint8_t i=0; i<sizeof(led); i++) {
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
    
    if (btnUp.wasReleased()) {
        msLast = ms;
        br[l] = (br[l] << 1) + 1;
        analogWrite(led[l], br[l]);
    }
    else if (btnDn.wasReleased()) {
        msLast = ms;
        br[l] = br[l] >> 1;
        if (br[l] == 0) br[l] = 1;
        analogWrite(led[l], br[l]);
    }
    else if (btnUp.pressedFor(LONG_PRESS)) {
        msLast = ms;
        analogWrite(led[l], 0);
        if (++l >= sizeof(led)) l = 0;
        analogWrite(led[l], br[l]);
        while (btnUp.isPressed()) btnUp.read();
    }
    else if (btnDn.pressedFor(LONG_PRESS)) {
        ledsOff();
        while (!btnDn.wasReleased()) btnDn.read();
        goToSleep();
    }
    else if (ms - msLast >= SLEEP) {
        ledsOff();
        goToSleep();
    }
}

void goToSleep(void)
{
    byte adcsra, mcucr1, mcucr2;

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
    delay(1);
    analogWrite(led[l], br[l]);
    ms = millis();
    msLast = ms;

    btnUp.read();
    btnDn.read();
    while (!btnDn.wasReleased() && !btnUp.wasReleased()) {
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

//turn off the LED and the regulator
void ledsOff(void)
{
    for (uint8_t i=0; i<sizeof(led); i++) {
        digitalWrite(led[i], LOW);
    }
    digitalWrite(REG_EN, LOW);
    delay(5);
}

