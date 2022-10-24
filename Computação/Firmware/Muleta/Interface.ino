
struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

Button button1 = {34, 0, false};
Button button2 = {35, 0, false};

byte display_map[10][7] = {
  {0,0,0,0,0,0,1},  //0
  {1,0,0,1,1,1,1},  //1
  {0,0,1,0,0,1,0},  //2
  {0,0,0,0,1,1,0},  //3
  {1,0,0,1,1,0,0},  //4
  {0,1,0,0,1,0,0},  //5
  {0,1,0,0,0,0,0},  //6
  {0,0,0,1,1,1,1},  //7
  {0,0,0,0,0,0,0},  //8
  {0,0,0,0,1,0,0}   //9
};

void display_num(int num)
{
  for (int i = 0;i < 7;i++) 
  {
     digitalWrite(display_pins[i], display_map[num][i]); 
  }
}