#define right_encoder_phaseA 3
#define right_encoder_phaseB 5
#define left_encoder_phaseA 2
#define left_encoder_phaseB 4

volatile unsigned long right_encoder_total = 0;
volatile unsigned long left_encoder_total = 0;

void setup()
{
  pinMode(right_encoder_phaseA, INPUT);
  pinMode(right_encoder_phaseB, INPUT);

  pinMode(left_encoder_phaseA, INPUT);
  pinMode(left_encoder_phaseB, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(right_encoder_phaseA),
    rightEncoderCallback,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(left_encoder_phaseA),
    leftEncoderCallback,
    RISING
  );

  Serial.begin(115200);
}

void loop()
{
  static unsigned long last_print = 0;

  if (millis() - last_print >= 500)
  {
    noInterrupts();
    unsigned long right_count = right_encoder_total;
    unsigned long left_count = left_encoder_total;
    interrupts();

    Serial.print("Right: ");
    Serial.print(right_count);

    Serial.print("    Left: ");
    Serial.println(left_count);
    

    last_print = millis();
  }

  // 傳 r 可以歸零
  if (Serial.available())
  {
    char c = Serial.read();

    if (c == 'r')
    {
      noInterrupts();
      right_encoder_total = 0;
      left_encoder_total = 0;
      interrupts();

      Serial.println("RESET");
    }
  }
}

void rightEncoderCallback()
{
  right_encoder_total++;
}

void leftEncoderCallback()
{
  left_encoder_total++;
}