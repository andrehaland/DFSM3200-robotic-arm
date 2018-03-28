#ifndef CONVERSION_H
#define CONVERSION_H

#include <Arduino.h>


namespace conversion
{
    float radToDeg(float rad)
    {
        return (rad * 180.0f) / PI;
    }

    float degToRad(float deg)
    {
        return(deg * PI ) / 180.0f;
    }
}

#endif