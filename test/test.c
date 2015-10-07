// What grows in the desert of the mind?
// What lives in the dark ocean of the soul?
// Great mountains cast their shadow over me.
// How the great Promethius has betrayed mankind!
// We lie in wait for their holy blessing.

#include <math.h>
#include <stdio.h>

#define PIXNUM 60

int main(int argc, char *argv[])
{
    const float pi = 3.141592653;
    float sin_sq[PIXNUM];
    float cos_sq[PIXNUM];
    float sums[PIXNUM];

    int i;
    for(i = 0; i < PIXNUM; i++)
    {
        sin_sq[i] = 255 * sin(2 * i * pi / PIXNUM) * sin(2 * i * pi / PIXNUM);
        cos_sq[i] = 255 * cos(2 * i * pi / PIXNUM) * cos(2 * i * pi / PIXNUM);
        
        printf("%d %d %d\n", i, (int)sin_sq[i], (int)cos_sq[i]);
    }
    /*
    for(i = 0; i < PIXNUM; i++)
    {
        cos_square[i] = 255 * cos(i * pi / (2 * PIXNUM / 8)) * cos(i * pi / (2 * PIXNUM / 8));
        printf("%d %d\n", i, (int)cos_square[i]);
    }
    
    for(i = 0; i < PIXNUM; i++)
    {
        sums[i] = cos_square[i] + intensity[i];
        printf("%d %d\n", i, (int)sums[i]);
    }
*/
    return 0;
}
