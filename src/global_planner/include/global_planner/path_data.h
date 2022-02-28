#ifndef PATH_DATA_H
#define PATH_DATA_H

#include <array>

class pathPoint
{
    public:
        pathPoint(){}
        ~pathPoint(){}

        std::array<int, 2> xy;
        float heading;
};



#endif