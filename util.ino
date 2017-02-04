// put the mcu to sleep
void gotoSleep(void)
{
    updateParams();                             //update eeprom
    uint8_t adcsra = ADCSRA;                    //save ADCSRA
    ADCSRA &= ~_BV(ADEN);                       //disable ADC
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    PCMSK0 |= _BV(PCINT1) | _BV(PCINT0);        //enable pin change interrupts
    GIMSK |= _BV(PCIE0);
    cli();                                      //stop interrupts to ensure the BOD timed sequence executes as required
    uint8_t mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);    //turn off the brown-out detector
    uint8_t mcucr2 = mcucr1 & ~_BV(BODSE);      //if the MCU does not have BOD disable capability,
    MCUCR = mcucr1;                             // this code has no effect
    MCUCR = mcucr2;
    sei();                                      //ensure interrupts enabled so we can wake up again
    digitalWrite(REG_EN, LOW);                  //regulator off
    sleep_cpu();                                //go to sleep
    sleep_disable();                            //wake up here
    ADCSRA = adcsra;                            //restore ADCSRA

    digitalWrite(REG_EN, HIGH);                 //regulator on
    delay(10);
    analogWrite(LED[l], br[l]);                 //restore last LED setting
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

// turn off the LEDs, optional fade
void ledsOff(bool fade)
{
    if (fade)
    {
        //calculate a fade interval: longer intervals for lower brightness
        uint8_t nbits(0);
        for (uint8_t i=0; i<8; i++)    //count the one bits in the brightness
        {
            if (br[l] & 1 << i) ++nbits;
        }
        uint32_t d = 1 << (8 - nbits);

        //fade the current LED
        for (uint8_t b=br[l]; b>0; b--)
        {
            analogWrite(LED[l], b);
            delay(d);
        }
    }

    //ensure both are off
    for (uint8_t i=0; i<sizeof(LED); i++)
    {
        digitalWrite(LED[i], LOW);
    }
}

//read 1.1V reference against Vcc
//from http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
int readVcc(void)
{
    ADMUX = _BV(MUX5) | _BV(MUX0);
    delay(10);                                //Vref settling time
    ADCSRA |= _BV(ADSC);                      //start conversion
    loop_until_bit_is_clear(ADCSRA, ADSC);    //wait for it to complete
    return 1126400L / ADC;                    //calculate AVcc in mV (1.1 * 1000 * 1024)
}

//read variables from eeprom into sram variables
void readParams(void)
{
    l = eeprom_read_byte( &l_ee );
    eeprom_read_block(br, &br_ee[0], sizeof(br));
    sleepInterval = eeprom_read_dword(&sleepInterval_ee);
    signature = eeprom_read_dword(&signature_ee);
    if (signature != TORCH_SIG) initParams();
}

// update the eeprom variables
void updateParams(void)
{
    eeprom_update_byte(&l_ee, l);
    eeprom_update_block(br, br_ee, sizeof(br));
    eeprom_update_dword(&sleepInterval_ee, sleepInterval);
    eeprom_update_dword(&signature_ee, signature);
}

// initialize the eeprom variables
void initParams(void)
{
    l = 0;
    br[0] = br[1] = DEFAULT_BRIGHTNESS;
    sleepInterval = SHORT_POWEROFF;
    signature = TORCH_SIG;
    updateParams();
}

