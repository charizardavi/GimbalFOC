#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6, 7);

float target_angle = 0;
float target_offset = 0;

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
void doP_Angle(char* cmd) { command.scalar(&motor.P_angle.P, cmd); }
void onMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);

    sensor.init();
    motor.linkSensor(&sensor);

    driver.voltage_power_supply = 15;
    driver.init();
    motor.linkDriver(&driver);

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::angle;
    motor.voltage_limit = 15;
    motor.LPF_velocity.Tf = 0.01f;
    motor.PID_velocity.P = .25f;
    motor.PID_velocity.I = 0;
    motor.PID_velocity.D = 0;
    motor.P_angle.P = 35.0F;
    motor.P_angle.I = 4.0F;
    motor.P_angle.D = 0.0F;
    motor.velocity_limit = 300;

    motor.useMonitoring(Serial);

    motor.init();
    motor.initFOC();

    command.add('T', doTarget, "target angle");
    command.add('P', doP_Angle, "angle controller P");
    command.add('M', onMotor, "full motor config");
    
    Serial.println(F("Motor ready."));
    _delay(1000);

    target_offset = motor.shaft_angle;
}

void playNoteNonBlocking(float frequency, int duration) {
    static unsigned long startTime = 0;
    static bool playing = false;
    static float period = 0;
    static bool state = false;
    
    if (!playing) {
        startTime = millis();
        period = 1000000.0 / frequency; // Microseconds per cycle
        playing = true;
    }
    
    if (playing) {
        if (micros() % (unsigned long)period < period / 2) {
            motor.move(target_offset + 1.0);
        } else {
            motor.move(target_offset - 1.0);
        }
        
        if (millis() - startTime >= duration) {
            playing = false;
        }
    }
}

void playMelody() {
    static int noteIndex = 0;
    static unsigned long lastNoteTime = 0;
    static bool playingMelody = false;
    
    float notes[] = {261.63, 293.66, 329.63, 349.23, 392.00, 440.00, 493.88}; // C4 to B4
    int duration = 300; // Milliseconds per note
    
    if (!playingMelody) {
        noteIndex = 0;
        playingMelody = true;
        lastNoteTime = millis();
    }
    
    if (playingMelody) {
        if (millis() - lastNoteTime >= duration + 50) { // 50ms gap between notes
            noteIndex++;
            lastNoteTime = millis();
        }
        
        if (noteIndex < 7) {
            playNoteNonBlocking(notes[noteIndex], duration);
        } else {
            playingMelody = false;
        }
    }
}

void loop() {
    motor.loopFOC();
    motor.move(target_angle + target_offset);
    command.run();
    

    playMelody();

}
