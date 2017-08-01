#include <PPMReader.h>
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

#define PPM_CHANNELS 16
#define DEADBAND 5
#define MID_RC 1500

#define RC_EXPO_LOOKUP_LENGTH 5
#define RC_EXPO 70
#define RC_RATE 90

int rcCommand[PPM_CHANNELS];
int rcData[PPM_CHANNELS];
int output[3];

#define MIXER_RULE_COUNT 5
#define SERVO_COUNT 3

typedef struct
{
    int8_t output;
    int8_t input;
    int8_t weight;
} mixer_t;

mixer_t mixer[MIXER_RULE_COUNT];

typedef struct
{
    int8_t rate;
} servoOutput_t;

servoOutput_t servoOutput[SERVO_COUNT];

Servo servoHardware[SERVO_COUNT];

PPMReader ppmReader(INPUT_PIN, INPUT_INTERRUPT);

int16_t lookupPitchRollRC[RC_EXPO_LOOKUP_LENGTH];

void setup()
{

#ifdef DEBUG
    Serial.begin(57600);
    Serial.println("ready");
#endif

    for(uint8_t i = 0; i < RC_EXPO_LOOKUP_LENGTH; i++) {
        lookupPitchRollRC[i] = (1526 + RC_EXPO * (i*i-15)) * i * (int32_t) RC_RATE / 1192;
    }

    /*
     * Mixer rules
     */
    mixer[0] = (mixer_t){OUTPUT_LEFT_AILERON, CHANNEL_ROLL, 50};
    mixer[1] = (mixer_t){OUTPUT_LEFT_AILERON, CHANNEL_PITCH, 50};
    mixer[2] = (mixer_t){OUTPUT_RIGHT_AILERON, CHANNEL_ROLL, -50};
    mixer[3] = (mixer_t){OUTPUT_RIGHT_AILERON, CHANNEL_PITCH, 50};
    mixer[4] = (mixer_t){OUTPUT_HOOK, CHANNEL_HOOK, 100};

    /*
     * Servo rules
     */
    servoOutput[OUTPUT_LEFT_AILERON] = (servoOutput_t){100};
    servoOutput[OUTPUT_RIGHT_AILERON] = (servoOutput_t){-100};
    servoOutput[OUTPUT_HOOK] = (servoOutput_t){100};

    servoHardware[0].attach(LEFT_AILERON_PIN);
    servoHardware[1].attach(RIGHT_AILERON_PIN);
    servoHardware[2].attach(HOOK_PIN);
}

void loop()
{
    static int count;
    uint16_t tmp,tmp2;

    while (ppmReader.get(count) != 0)
    {
        rcData[count] = constrain(ppmReader.get(count) - MID_RC, -500, 500);

#ifdef DEBUG
        Serial.print(ppmReader.get(count));
        Serial.print("  ");
#endif

        count++;
    }
    count = 0;

    /*
     * Process rcData to rcCommand
     */
    for (uint8_t i = 0; i < PPM_CHANNELS; i++)
    {

        if (i < 3)
        {
            /*
             * ROLL and PITCH
             */
            tmp = abs(rcData[i]);

            if (tmp > DEADBAND) {
                tmp -= DEADBAND;
            } else {
                tmp = 0;
            }

            tmp2 = tmp >> 7;
            rcCommand[i] = lookupPitchRollRC[tmp2] + ((tmp - (tmp2 << 7)) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) >> 7);
            
            if (rcData[i] < MID_RC) {
                rcCommand[i] = -rcCommand[i];
            }
        } else {
            /*
             * All other channels
             */
            rcCommand[i] = rcData[i];
        }
    }

    for (uint8_t i = 0; i < SERVO_COUNT; i++)
    {
        output[i] = 0;
    }

    /*
     * process rules
     */
    for (uint8_t i = 0; i < MIXER_RULE_COUNT; i++)
    {
        output[mixer[i].output] += rcCommand[mixer[i].input] * mixer[i].weight / 100;
    }

    for (uint8_t i = 0; i < SERVO_COUNT; i++)
    {
        servoHardware[i].writeMicroseconds((output[i] * servoOutput[i].rate / 100) + MID_RC);
    }

#ifdef DEBUG
    Serial.println("");
#endif

    delay(20); //you can even use delays!!!
}
