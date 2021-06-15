// Developed by @lucns

#include<Wire.h>
#include "Oled.h"
#include "Arduino.h"

#define DEFAULT_ADDRESS 0x3c
#define COMMAND 0x00
#define DATA 0x40

Oled::Oled(int _width, int _height) {
  _WIDTH = _width;
  _HEIGHT = _height;
}

void Oled::initialize() {
  const int configurations[] = {
    0x00, 0xae, 0xd5, 0x80, 0xa8, 0x1f, 0xd3, 0x00, 0x40, 0x8d, 0x14, 0xa1,
    0xc8, 0xda, 0x02, 0x81, 0x7f, 0xd9, 0xf1, 0xdb, 0x40, 0xa4, 0xa6, 0xaf
  };
  Wire.begin();
  Wire.beginTransmission(DEFAULT_ADDRESS);
  Wire.write(COMMAND);
  for (int i = 0; i < 24; i++) Wire.write(configurations[i]);
  Wire.endTransmission();

  clear(false);
}

void Oled::setPage(int page) {
  Wire.beginTransmission(DEFAULT_ADDRESS);
  Wire.write(COMMAND);
  Wire.write(0XB0 | page);
  Wire.endTransmission();
}

void Oled::setColumn(int column) {
  Wire.beginTransmission(DEFAULT_ADDRESS);
  Wire.write(COMMAND);
  Wire.write(0x00 | (column & 0XF));
  Wire.write(0x10 | (column >> 4));
  Wire.endTransmission();
}

void Oled::setContrast(int contrast) {
  Wire.beginTransmission(DEFAULT_ADDRESS);
  Wire.write(COMMAND);
  Wire.write(0x81);
  Wire.write(contrast);
  Wire.endTransmission();
}

void Oled::apply() {
  unsigned int twbrbackup = TWBR;
  TWBR = 12;
  for (int j = 0; j < _HEIGHT / 8; j++) {
    setPage(j);
    for (int i = 0; i < _WIDTH; i++) {
      setColumn(i);
      Wire.beginTransmission(DEFAULT_ADDRESS); // address
      Wire.write(DATA); // data stream
      Wire.write(RAM[i + (_WIDTH * j)]);
      Wire.endTransmission();
    }
  }
  TWBR = twbrbackup;
}

void Oled::clear(bool enable) {
  for (int i = 0; i < (_HEIGHT / 8) * _WIDTH; i++) RAM[i] = enable ? 0xFF : 0x00;
}

void Oled::drawRam(int x, int y, int enable) {
  int page, rest;
  if (y > 7) {
    page = y / 8;
    rest = y % 8;
  } else {
    rest = y;
    page = 0;
  }
  setColumn(x);
  setPage(page);
  int index = (page * _WIDTH) + x;
  RAM[index] ^= (-enable ^ RAM[index]) & (1UL << rest);
}

void Oled::drawPixel(int x, int y, int enable) {
  drawRam(x, y, enable);
}

void Oled::drawLine(int x1, int y1, int x2, int y2) {
  int cx = x1;
  int cy = y1;
  int dx = x2 - cx;
  int dy = y2 - cy;
  int sx = 0;
  int sy = 0;

  if (dx < 0) dx = 0 - dx;
  if (dy < 0) dy = 0 - dy;

  if (cx < x2) sx = 1;
  else sx = -1;
  if (cy < y2) sy = 1;
  else sy = -1;
  int err = dx - dy;

  for (int n = 0; n < _WIDTH * _HEIGHT; n++) {
    drawRam(cx, cy, true);
    if (cx == x2 && cy == y2) break;
    int e2 = 2 * err;
    if (e2 > 0 - dy) {
      err = err - dy;
      cx = cx + sx;
    }
    if (e2 < dx) {
      err = err + dx;
      cy = cy + sy;
    }
  }
}

void Oled::drawBitmap(int x, int y, int w, int h, int *bitmap, bool flip) {
  if (flip) {
    int buff[w * (h / 8)];
    for (int i = 0; i < h / 8; i++) {
      for (int a = 0; a < w; a++) buff[(i * w) + a] = bitmap[w - a + (i * w) - 1];
    }
    drawBitmap(x, y, w, h, buff);
  } else {
    drawBitmap(x, y, w, h, bitmap);
  }
}

void Oled::drawBitmap(int x, int y, int w, int h, int *bitmap) {
  for (int page = 0; page < h / 8; page++) {
    for (int column = 0; column < w; column++) {
      for (int bitP = 0; bitP < 8; bitP++) {
        int enable = (bitmap[column + (page * w)] >> bitP) & 1U;
        drawRam(column + x, (page * 8) + bitP + y, enable);
      }
    }
  }
}
