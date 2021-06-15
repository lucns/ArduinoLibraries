// Developed by @lucns

class Oled {
  private:
    int _WIDTH, _HEIGHT, RAM[0];
    void setPage(int page);
    void setColumn(int column);
    void drawRam(int x, int y, int enable);
  public:
    Oled(int width, int height);
    void initialize();
    void apply();
    void setContrast(int contrast);
    void clear(bool enable = false);
    void drawPixel(int x, int y, int enable = 1);
    void drawLine(int x, int y, int x2, int y2);
    void drawBitmap(int x, int y, int w, int h, int *bitmap);
    void drawBitmap(int x, int y, int w, int h, int *bitmap, bool flip);
};
