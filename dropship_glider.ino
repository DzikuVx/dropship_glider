/*
 * PPM decoder based on https://code.google.com/archive/p/read-any-ppm/
 */
#include <Servo.h>

#define LEFT_AILERON_PIN 9
#define RIGHT_AILERON_PIN 10
#define HOOK_PIN 11

#define CHANNEL_ROLL 0
#define CHANNEL_PITCH 1
#define CHANNEL_HOOK 4

#define OUTPUT_LEFT_AILERON 0
#define OUTPUT_RIGHT_AILERON 1
#define OUTPUT_HOOK 2

#define INPUT_PIN 2
#define INPUT_INTERRUPT 1

int ppm[16]; //array for storing up to 16 servo signals
int rcCommand[16];
int output[3];

#define MIXER_RULE_COUNT 5
#define SERVO_COUNT 3

typedef struct {
    int8_t output;    
    int8_t input;
    int8_t weight;
} mixer_t;

mixer_t mixer[MIXER_RULE_COUNT];

typedef struct {
    int8_t rate;
} servoOutput_t;

servoOutput_t servoOutput[SERVO_COUNT];

Servo servoHardware[SERVO_COUNT];

void setup() {
    Serial.begin(57600);
    Serial.println("ready");

    /*
     * Mixer rules
     */
    mixer[0] = (mixer_t) {OUTPUT_LEFT_AILERON, CHANNEL_ROLL, 50};
    mixer[1] = (mixer_t) {OUTPUT_LEFT_AILERON, CHANNEL_PITCH, 50};
    mixer[2] = (mixer_t) {OUTPUT_RIGHT_AILERON, CHANNEL_ROLL, -50};
    mixer[3] = (mixer_t) {OUTPUT_RIGHT_AILERON, CHANNEL_PITCH, 50};
    mixer[4] = (mixer_t) {OUTPUT_HOOK, CHANNEL_HOOK, 100};

    /*
     * Servo rules
     */
    servoOutput[OUTPUT_LEFT_AILERON] = (servoOutput_t) {100};
    servoOutput[OUTPUT_RIGHT_AILERON] = (servoOutput_t) {-100};
    servoOutput[OUTPUT_HOOK] = (servoOutput_t) {100};

    servoHardware[0].attach(LEFT_AILERON_PIN);
    servoHardware[1].attach(RIGHT_AILERON_PIN);
    servoHardware[2].attach(HOOK_PIN);

    pinMode(INPUT_PIN, INPUT);
    attachInterrupt(INPUT_INTERRUPT, read_ppm, CHANGE);
}

void read_ppm() {
    static unsigned int pulse;
    static unsigned long counter;
    static byte channel;
    static unsigned long previousCounter = 0;
    static unsigned long currentMicros = 0;

    currentMicros = micros();
    counter = (currentMicros - previousCounter) * 2;
    previousCounter = currentMicros;

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

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        output[i] = 0;
    }

    /*
     * process rules
     */
    for (uint8_t i = 0; i < MIXER_RULE_COUNT; i++) {
        output[mixer[i].output] += rcCommand[mixer[i].input] * mixer[i].weight / 100;
    }

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        servoHardware[i].writeMicroseconds((output[i] * servoOutput[i].rate / 100) + 1500);
    }

    Serial.println("");
    delay(100); //you can even use delays!!!
}
