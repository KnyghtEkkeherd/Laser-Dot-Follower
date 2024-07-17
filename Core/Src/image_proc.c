#include "image_proc.h"
#include <math.h>
extern  uint8_t frame[100][320];
extern int target;


float max(float a, float b, float c) { return ((a > b)? (a > c ? a : c) : (b > c ? b : c));}

float min(float a, float b, float c) { return ((a < b)? (a < c ? a : c) : (b < c ? b : c));}

uint8_t rgb_to_hsv(float r, float g, float b) {
    // R, G, B values are divided by 63 and 31 because of the RGB565 standard
    // to change the range from 0..63 and 0...31 to 0..1:
    float h = 0, s = 0, v = 0;
    r /= 31.0; g /= 63.0; b /= 31.0;
    float cmax = max(r, g, b);
    float cmin = min(r, g, b);
    float diff = cmax-cmin;
    if (cmax == cmin){h = 0;}
    else if (cmax == r){h = fmod((60 * ((g - b) / diff) + 360), 360.0);}
    else if (cmax == g){h = fmod((60 * ((b - r) / diff) + 120), 360.0);}
    else if (cmax == b){h = fmod((60 * ((r - g) / diff) + 240), 360.0);}
    // handle cmax = 0
    if (cmax == 0){s = 0;}
    else{s = (diff / cmax);}
    // compute v
    v = cmax;

    // Normalize the hue to 0...1 range
    h /= 360.0;

    // Thresholding method from IEEE Xplore
    // Apply thresholding for H-value:
    if (0.910 <= h && h <= 0.95){//0.056 IEEE
        // Apply thresholding for S-value:
        if (0 <= s && s<= 0.5){
            // Apply thresholiding for V-value:
            if (0.8 <= v && v <=1){
                return 0xffff;
            }
        }
    }
    return 0;
}

void getMiddleDistance(void){
    // Calcualte the center of max of the image
    int x_mean = 0;
    int x_sum = 0;
    uint16_t total = 0;

    // Get total pixel value:
    for (int i=0; i<100; i++){
        for (int j=0; j<320; j++){
            total = total + frame[i][j];
        }
    }
    // // Take the weighted average
    for (int i=0; i<100; i++){
        for (int j=0; j<320; j++){
            x_sum = x_sum + j*frame[i][j];
        }
    }
    //handle division by 0 if there is no laser dot in the image
    if (total == 0){target = -200; return;} // laser not in the iamge

    x_mean = x_sum / total;
    target = (160-x_mean);
}
