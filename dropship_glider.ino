/*
 * PPM decoder based on https://code.google.com/archive/p/read-any-ppm/
 */
#include <Servo.h>

#define LEFT_AILERON_PIN 9
#define RIGHT_AILERON_PIN 10
#define HOOK_PIN 11

#define CHANNEL_ROLL 1
#define CHANNEL_PITCH 2
#define CHANNEL_HOOK 5

#define INPUT_PIN 2
#define INPUT_INTERRUPT 0
#define INPUT_CHANNELS 6

int ppm[16]; //array for storing up to 16 servo signals
int rcCommand[16];
int output[3];

Servo servoLeft;
Servo servoRight;
Servo servoHook;

void setup() {
    Serial.begin(57600);
    Serial.println("ready");

    pinMode(INPUT_PIN, INPUT);
    attachInterrupt(INPUT_INTERRUPT, read_ppm, CHANGE);

    TCCR1A = 0x00; // COM1A1=0, COM1A0=0 => Disconnect Pin OC1 from Timer/Counter 1 -- PWM11=0,PWM10=0 => PWM Operation disabled
    TCCR1B = B00000010; //0x02;	   // 16MHz clock with prescaler means TCNT1 increments every .5 uS (cs11 bit set
    TIMSK1 = _BV(ICIE1); // enable input capture interrupt for timer 1

    servoLeft.attach(LEFT_AILERON_PIN);
    servoRight.attach(RIGHT_AILERON_PIN);
    servoHook.attach(HOOK_PIN);
}

void read_ppm() {
    static unsigned int pulse;
    static unsigned long counter;
    static byte channel;

    counter = TCNT1;
    TCNT1 = 0;

    if (counter < 1020) { //must be a pulse
        pulse = counter;
    } else if (counter > 3820) { //sync
        channel = 0;
    } else { //servo values between 810 and 2210 will end up here
        ppm[channel] = (counter + pulse) / 2;
        channel++;
    }
}

void loop() {
    //You can delete everithing inside loop() and put your own code here
    static int count;

    while (ppm[count] != 0) { //print out the servo values
        rcCommand[count] = ppm[count] - 1500;
        Serial.print(ppm[count]);
        Serial.print("  ");
        count++;
    }
    count = 0;

    /*
     * Left aileron
     */
    output[0] = (ppm[CHANNEL_ROLL] >> 1) + (ppm[CHANNEL_PITCH] >> 1);

    /*
     * Right aileron
     */
    output[1] = (ppm[CHANNEL_PITCH] >> 1) - (ppm[CHANNEL_ROLL] >> 1);

    /*
     * Hook channel
     */
    output[2] = ppm[CHANNEL_HOOK];

    servoLeft.writeMicroseconds(output[0] + 1500);
    servoRight.writeMicroseconds(output[1] + 1500);
    servoHook.writeMicroseconds(output[2]);

    Serial.println("");
    delay(100); //you can even use delays!!!
}