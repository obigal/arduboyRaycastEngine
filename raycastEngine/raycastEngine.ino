#include <Arduboy2.h>
//#include <Arduboy2Audio.h>
#include <Arduboy2Core.h>
//#include <Sprites.h>

#define MAP_WIDTH 15
#define MAP_HEIGHT 15

Arduboy2 ab;
const uint8_t PROGMEM worldMap[MAP_WIDTH][MAP_HEIGHT] = {
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},
  {1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};
double posX = 13.0, posY = 7.0, posZ = 0; //player position
int moveLeft = 0, moveRight = 0, moveFwd = 0, moveBck = 0, rotateRt = 0, rotateLt = 0; //player direction vect
double dirX = -1.0, dirY = 0.0, planeX = 0.0, planeY = 0.66;//player vect vars
double speedVar = 0.40, rotSpeedVar = 0.2, rotateSpeedX = 0.0045, rotateSpeedY = 1.5; // player movement speed
double curTime = 0, prevTime = 0;

int handleInput() {
  if (ab.pressed(LEFT_BUTTON)) {
    moveLeft = 1;
  }
  if (ab.pressed(RIGHT_BUTTON)) {
    moveRight = 1;
  }
  if (ab.pressed(UP_BUTTON)) {
    moveFwd = 1;
  }
  if (ab.pressed(DOWN_BUTTON)) {
    moveBck = 1;
  }
  if (ab.pressed(A_BUTTON)) {
    rotateLt = 1;
  }
  if (ab.pressed(B_BUTTON)) {
    rotateRt = 1;
  }

  if (ab.notPressed(LEFT_BUTTON)) {
    moveLeft = 0;
  }
  if (ab.notPressed(RIGHT_BUTTON)) {
    moveRight = 0;
  }
  if (ab.notPressed(UP_BUTTON)) {
    moveFwd = 0;
  }
  if (ab.notPressed(DOWN_BUTTON)) {
    moveBck = 0;
  }
  if (ab.notPressed(A_BUTTON)) {
    rotateLt = 0;
  }
  if (ab.notPressed(B_BUTTON)) {
    rotateRt = 0;
  }
  return 1;
}

void movePlayer (double deltaTime) {
  if (moveFwd) {
    double moveStepX = dirX * deltaTime * speedVar;
    double moveStepY = dirY * deltaTime * speedVar;

    if (!pgm_read_byte(&worldMap[(int)(posX + moveStepX)][(int)posY]))
      posX += moveStepX;
    if (!pgm_read_byte(&worldMap[(int)(posX)][(int)(posY + moveStepY)]))
      posY += moveStepY;
  }
  if (moveBck) {
    double moveStepX = -dirX * deltaTime * speedVar;
    double moveStepY = -dirY * deltaTime * speedVar;

    if (!pgm_read_byte(&worldMap[(int)(posX + moveStepX)][(int)posY]))
      posX += moveStepX;
    if (!pgm_read_byte(&worldMap[(int)(posX)][(int)(posY + moveStepY)]))
      posY += moveStepY;
  }
  if (moveRight) {
    //Find perp. vector by switching x and y, and negating one
    double moveStepX = dirY * deltaTime * speedVar;
    double moveStepY = -dirX * deltaTime * speedVar;

    if (!pgm_read_byte(&worldMap[(int)(posX + moveStepX)][(int)posY]))
      posX += moveStepX;
    if (!pgm_read_byte(&worldMap[(int)(posX)][(int)(posY + moveStepY)]))
      posY += moveStepY;
  }
  if (moveLeft) {
    double moveStepX = -dirY * deltaTime * speedVar;
    double moveStepY = dirX * deltaTime * speedVar;

    if (!pgm_read_byte(&worldMap[(int)(posX + moveStepX)][(int)posY]))
      posX += moveStepX;
    if (!pgm_read_byte(&worldMap[(int)(posX)][(int)(posY + moveStepY)]))
      posY += moveStepY;
  }
  if (rotateRt) {
    double newRotateSpeed = -rotSpeedVar;
    double oldDirX = dirX;
    dirX = dirX * cos(newRotateSpeed) - dirY * sin(newRotateSpeed);
    dirY = oldDirX * sin(newRotateSpeed) + dirY * cos(newRotateSpeed);
    double oldPlaneX = planeX;
    planeX = planeX * cos(newRotateSpeed) - planeY * sin(newRotateSpeed);
    planeY = oldPlaneX * sin(newRotateSpeed) + planeY * cos(newRotateSpeed);
  }
  if (rotateLt) {
    double newRotateSpeed = rotSpeedVar;
    double oldDirX = dirX;
    dirX = dirX * cos(newRotateSpeed) - dirY * sin(newRotateSpeed);
    dirY = oldDirX * sin(newRotateSpeed) + dirY * cos(newRotateSpeed);
    double oldPlaneX = planeX;
    planeX = planeX * cos(newRotateSpeed) - planeY * sin(newRotateSpeed);
    planeY = oldPlaneX * sin(newRotateSpeed) + planeY * cos(newRotateSpeed);
  }
}

void renderWalls() {
  for (int x = 0; x < WIDTH; ++x) {
    double screenCoordinate = (x << 1) / (double)WIDTH - 1;
    double rayPosX = posX;
    double rayPosY = posY;
    double rayDirX = dirX + planeX * screenCoordinate;
    double rayDirY = dirY + planeY * screenCoordinate;

    int mapX = (int) rayPosX;
    int mapY = (int) rayPosY;

    double sideDistX;
    double sideDistY;

    double deltaX = sqrt(1 + (rayDirY * rayDirY) / (rayDirX * rayDirX));
    double deltaY = sqrt(1 + (rayDirX * rayDirX) / (rayDirY * rayDirY));
    double wallDist;

    int stepX;
    int stepY;

    int nsWall = false;

    if (rayDirX < 0) {
      stepX = -1;
      sideDistX = (rayPosX - mapX) * deltaX;
    }
    else {
      stepX = 1;
      sideDistX = (mapX + 1.0 - rayPosX) * deltaX;
    }

    if (rayDirY < 0) {
      stepY = -1;
      sideDistY = (rayPosY - mapY) * deltaY;
    }
    else {
      stepY = 1;
      sideDistY = (mapY + 1.0 - rayPosY) * deltaY;
    }

    while (!pgm_read_byte(&worldMap[mapX][mapY])) {
      if (sideDistX < sideDistY) {
        sideDistX += deltaX;
        mapX += stepX;
        nsWall = false;
      }
      else {
        sideDistY += deltaY;
        mapY += stepY;
        nsWall = true;
      }
      if (mapX > MAP_WIDTH || mapY > MAP_HEIGHT)
        return;
    }

    if (nsWall)
      wallDist = fabs((mapY - rayPosY + (1 - stepY) / 2) / rayDirY);
    else
      wallDist = fabs((mapX - rayPosX + (1 - stepX) / 2) / rayDirX);

    int wallHeight = abs((int)(WIDTH / wallDist));

    int start = -wallHeight / 2 + HEIGHT / 2;
    if (start < -posZ)
      start = -posZ;

    int endH = wallHeight / 2 + HEIGHT / 2;
    if (endH > HEIGHT - posZ)
      endH = HEIGHT - posZ;

    int y1 = start + posZ;
    if (y1 < 0)
      y1 = 0;
    int y2 = endH + posZ;
    if (y2 < 0)
      y2 = 0;
    int h = y2 - y1;
    //drawLine(x,y1,x,y2);
    //ab.drawFastVLine(x, y1, h);
    DrawVLine(x, y1, y1+h, 0xff);
  }
}

// Adpated from https://github.com/a1k0n/arduboy3d/blob/master/draw.cpp
// since the AVR has no barrel shifter, we'll do a progmem lookup
const uint8_t topmask_[] PROGMEM = {
  0xff, 0xfe, 0xfc, 0xf8, 0xf0, 0xe0, 0xc0, 0x80
};
const uint8_t bottommask_[] PROGMEM = {
  0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7f, 0xff
};

void DrawVLine(uint8_t x, int8_t y0_, int8_t y1_, uint8_t pattern)
{
  uint8_t *screenptr = arduboy.getBuffer() + x;

  if (y1_ < y0_ || y1_ < 0 || y0_ > 63) return;

  // clip (FIXME; clipping should be handled elsewhere)
  // cast to unsigned after clipping to simplify generated code below
  uint8_t y0 = y0_, y1 = y1_;
  if (y0_ < 0) y0 = 0;
  if (y1_ > 63) y1 = 63;

  uint8_t *page0 = screenptr + ((y0 & 0x38) << 4);
  uint8_t *page1 = screenptr + ((y1 & 0x38) << 4);
  if (page0 == page1)
  {
    uint8_t mask = pgm_read_byte(topmask_ + (y0 & 7))
                   & pgm_read_byte(bottommask_ + (y1 & 7));
    *page0 &= ~mask;
    *page0 |= pattern & mask;  // fill y0..y1 in same page in one shot
  }
  else
  {
    uint8_t mask = pgm_read_byte(topmask_ + (y0 & 7));
    *page0 &= ~mask;
    *page0 |= pattern & mask;  // write top 1..8 pixels
    page0 += 128;
    while (page0 != page1)
    {
      *page0 = pattern;  // fill middle 8 pixels at a time
      page0 += 128;
    }
    mask = pgm_read_byte(bottommask_ + (y1 & 7));  // and bottom 1..8 pixels
    *page0 &= ~mask;
    *page0 |= pattern & mask;
  }
}

void DrawBackground()
{
  uint8_t* ptr = ab.getBuffer();
  uint8_t counter = 32;

  while (counter--)
  {
    *ptr++ = 0x04;  *ptr++ = 0;
    *ptr++ = 0;  *ptr++ = 0;
    *ptr++ = 0;  *ptr++ = 0;
    *ptr++ = 0;  *ptr++ = 0;
    *ptr++ = 0x20;  *ptr++ = 0;
    *ptr++ = 0;  *ptr++ = 0;
    *ptr++ = 0;  *ptr++ = 0;
    *ptr++ = 0;  *ptr++ = 0;
  }

  counter = 64;
  while (counter--)
  {
    *ptr++ = 0x55;  *ptr++ = 0xAA;
    *ptr++ = 0x55;  *ptr++ = 0xAA;
    *ptr++ = 0x55;  *ptr++ = 0xAA;
    *ptr++ = 0x55;  *ptr++ = 0xAA;
  }
}

void setup() {
  // put your setup code here, to run once:
  ab.begin();
  ab.setFrameRate(60);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!(ab.nextFrame()))
    return;

  ab.clear();
  handleInput();
  DrawBackground();
  renderWalls();
  movePlayer(1);
  ab.display();
}
