// Pins
#define PWM_PIN_IN1 33     // Driver In 1
#define PWM_PIN_IN2 32     // Driver In 2

// PWM configurations
#define PWM_FRQ 5000   // PWM frequency (Hz)
#define PWM_RES 8      // PWM resolution (bits)
#define PWM_CHNL0 0     // PWM channel for in 1
#define PWM_CHNL1 1     // PWM channel for in 2

float vel = 0.0;

void setup() {
  Serial.begin(115200);
  // Configure PWM
  // Set pins as output
  pinMode(PWM_PIN_IN1, OUTPUT); 
  pinMode(PWM_PIN_IN2, OUTPUT);
  // Configure pwm channel
  ledcSetup(PWM_CHNL0, PWM_FRQ, PWM_RES); 
  ledcSetup(PWM_CHNL1, PWM_FRQ, PWM_RES);
  // Assign pins
  ledcAttachPin(PWM_PIN_IN1, PWM_CHNL0);
  ledcAttachPin(PWM_PIN_IN2, PWM_CHNL1);

}

void loop() {
  if (Serial.available()>0) {
    vel = Serial.parseInt();
    vel = constrain(vel, -255, 255);
    Serial.print("Vel: "); Serial.println(vel);

    if (vel >= 0) {
    ledcWrite(PWM_CHNL0, vel); //
    ledcWrite(PWM_CHNL1, 0); // set to 0
    }else{
      ledcWrite(PWM_CHNL1, vel); //
      ledcWrite(PWM_CHNL0, 0); // set to 0
    }

  }

}