#include <SPI.h>
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();
TFT_eSPI_Button key[2];

void setup() {
  Serial.begin(115200);
  
  tft.init();

  tft.setRotation(0);
  
  uint16_t calData[5] = { 315, 3498, 238, 3474, 2 };
  tft.setTouch(calData);
  
  tft.fillScreen(TFT_BLACK);

  drawButtons(0,60,50,100,80,"Hello");
  drawButtons(1,180,50,100,80,"Hello2");
}

void loop() {
  uint16_t t_x = 0, t_y = 0;

  bool pressed = tft.getTouch(&t_x, &t_y);

  // Adjust press state of each key appropriately
  for (uint8_t b = 0; b < 2; b++) {
    if (pressed && key[b].contains(t_x, t_y)) 
      key[b].press(true);  // tell the button it is pressed
    else
      key[b].press(false);  // tell the button it is NOT pressed
  }

  if(key[0].justPressed()) {
    Serial.println("lol");
  }
  if(key[1].justPressed()) {
    Serial.println("lol2");
  }
}

void drawButtons(int id, int x, int y, int w, int h, String text)
{
    key[id].initButton(&tft,
                      x,
                      y, // x, y, w, h, outline, fill, text
                      w,
                      h,
                      TFT_GREEN, // Outline
                      TFT_CYAN, // Fill
                      TFT_BLACK, // Text
                      "", // 10 Byte Label
                      1);

    // Adjust button label X delta according to array position
    // setLabelDatum(uint16_t x_delta, uint16_t y_delta, uint8_t datum)
    //key[0].setLabelDatum(-25, 0, ML_DATUM);

    key[id].drawButton(false, text);
}
