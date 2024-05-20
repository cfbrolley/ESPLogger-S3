#ifndef Serial_Debug_h
#define Serial_Debug_h
#include <arduino.h>

class Serial_Debug
{
  public:
    Serial_Debug(int baud); //pass baud rate for serial monitor
    void begin();
    void debugdata(unsigned long _timer, float _pressure, float _altitude, float _correctedalt, float _ax, float _ay, float _az, float _gx, float _gy, float _gz, int _state); //pass all logger variables
    void debugBMP(int _BMPcode, float  _altioffset); //allows passing of an identifier to display different error messages
    void debugIMU(int _IMUcode);
    void debugSD(int _SDcode);
    void debuggeneric();

  private:
    int _baud, _BMPcode, _IMUcode, _SDcode;
    unsigned long _timer;
    float _pressure, _altitude, _altioffset, _correctedalt, _ax, _ay, _az, _gx, _gy, _gz;
};

#endif