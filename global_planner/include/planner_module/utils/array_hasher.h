// #include <map>

#ifndef ARRAY_HASHER_H
#define ARRAY_HASHER_H

struct ArrayHasher
{
    std::size_t operator()(const std::array<int, 2> &a) const
    {
        std::size_t h = 0;
        for (auto e : a)
        {
            h ^= std::hash<int>{}(e) + 0x9e3779b9 + (h << 6) + (h >> 2);
        }
        return h;
    }
};



#endif