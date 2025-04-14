//Board: STM32 MCU Based Boards / Generic STM32G0 series
//Board part number: Generic G030F6Px
//Upload metod: STM32 Cube Programmer (Serial)

#define LED_PIN   PB3

int message_period = 0;
int port_pwm = 0;
int port_pwm_dir = 0;


//--------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("KAmod BlackPico STM32G030, Hello :)");
  pinMode(LED_PIN, OUTPUT);
}

//--------------------------------------
void loop() {
  analogWrite(LED_PIN, (255 - port_pwm));
  if (port_pwm_dir == 0){
    port_pwm += 20;
    if (port_pwm >= 150){
      port_pwm = 150;
      port_pwm_dir = 1;
    }
  } else {
    port_pwm -= 20;
    if (port_pwm <= 0){
      port_pwm = 0;
      port_pwm_dir = 0;
    }
  }

  delay(100); 
}
