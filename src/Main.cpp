#include <stdio.h>
#include <random>
#include <chrono>

#include "QuadTree.h"

int main(int argc, char** argv)
{
    uint32 seed = 42;
    uint32 space_size = 1024;
    uint32 loops = 1e6;

    std::mt19937 gen;
    gen.seed(seed);

    std::uniform_int_distribution<> dis(0, space_size - 1);

    typedef std::chrono::high_resolution_clock Clock;

    Clock::time_point t1 = Clock::now();

    // run measured code
    {
        PRQuadTree tree(space_size, space_size);

        for (uint32 i = 0; i < loops; ++i)
        {
            uint32 x = dis(gen);
            uint32 y = dis(gen);
            Point p(x, y);

            tree.Insert(p);
        }
    }

    Clock::time_point t2 = Clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

    printf("PRQuadTree - %u inserts took %u ms\n",
        loops, uint32(duration));
}
