#ifndef PATH_DATA_H
#define PATH_DATA_H

#include <array>

class pathPoint
{
    public:
        pathPoint(){};
        // pathPoint(float x, float y, float head);
        
        ~pathPoint(){}

        // std::array<float, 2> xy;
        float x, y;
        float heading;
};


// pathPoint::pathPoint(float x, float y, float head) : heading{head}
// {
//     std::array<float,2> xandy {x,y};
//     xy = xandy;
// }

#endif