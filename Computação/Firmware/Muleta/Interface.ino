struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};

Button button1 = { 34, 0, false };
Button button2 = { 35, 0, false };

byte display_pins[] = { 27, 26, 25, 33, 32, 14, 13 };

//variables to keep track of the timing of recent interrupts
unsigned long button_time = 0;
unsigned long last_button_time = 0;

bool select_on = false;
uint16_t len_select = 0;

void IRAM_ATTR isr() {
  button_time = millis();
  if (button_time - last_button_time > 250) {
    if (digitalRead(35) == HIGH) {
      button1.numberKeyPresses++;
      button1.pressed = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    if (digitalRead(34) == HIGH) {
      button2.numberKeyPresses++;
      button2.pressed = true;
      digitalWrite(LED_BUILTIN, LOW);
    }
    last_button_time = button_time;
  }
}

byte display_map[10][7] = {
  { 0, 0, 0, 0, 0, 0, 1 },  //0
  { 1, 0, 0, 1, 1, 1, 1 },  //1
  { 0, 0, 1, 0, 0, 1, 0 },  //2
  { 0, 0, 0, 0, 1, 1, 0 },  //3
  { 1, 0, 0, 1, 1, 0, 0 },  //4
  { 0, 1, 0, 0, 1, 0, 0 },  //5
  { 0, 1, 0, 0, 0, 0, 0 },  //6
  { 0, 0, 0, 1, 1, 1, 1 },  //7
  { 0, 0, 0, 0, 0, 0, 0 },  //8
  { 0, 0, 0, 0, 1, 0, 0 }   //9
};

void display_num(int num) {
  for (int i = 0; i < 7; i++) {
    digitalWrite(display_pins[i], display_map[num][i]);
  }
}

void diplay_clear() {
  for (int i = 0; i < 7; i++) {
    digitalWrite(display_pins[i], HIGH);
  }
}

void Interface_Setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);

  pinMode(button1.PIN, INPUT);
  attachInterrupt(button1.PIN, isr, FALLING);
  pinMode(button2.PIN, INPUT);
  attachInterrupt(button2.PIN, isr, FALLING);
}

void Interface_Loop() {
  if (button2.pressed) {
    select_on = true;
    display_num(button2.numberKeyPresses % 10);
  }
  if (button1.pressed && select_on) {
    len_select = button2.numberKeyPresses % 10;
  } else if (millis() - last_button_time > 2000) {
    diplay_clear();
    select_on = false;
  }
  button1.pressed = false;
  button2.pressed = false;
}