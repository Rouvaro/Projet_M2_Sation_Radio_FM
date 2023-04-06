#include "SeeedGroveFMReceiver.cpp"

int main()
{
    i2c_init();
    SeeedGroveFMReceiver myFM;
    myFM.initSensor();
    myFM.setVolumeLevel(vol);
    myFM.setFMSeekDown();
    _delay_ms(2000);
    myFM.setFMSeekDown();
    _delay_ms(2000);
    
    return 0;
}