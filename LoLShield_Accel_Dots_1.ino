/*
 * LoL shield Accelerometer based on: https://learn.adafruit.com/animated-led-sand/code
 * Using my old Mini-IMU :   http://www.pololu.com/product/1265
 *                which uses an LSM303 accel / compass 
 * and a 9x14 Charliplexed LoL shield
*/

#include <Wire.h>
#include <LSM303.h>
#include "Charliplexing.h"      //initializes the LoL Sheild library

#define N_GRAINS     16 // Number of grains of sand
#define WIDTH        14 // Display width in pixels
#define HEIGHT        9 // Display height in pixels
#define MAX_FPS      32 // Maximum redraw rate, frames/second

// The 'sand' grains exist in an integer coordinate space that's 256X
// the scale of the pixel grid, allowing them to move and interact at
// less than whole-pixel increments.
#define MAX_X (WIDTH  * 256 - 1) // Maximum X coordinate in grain space
#define MAX_Y (HEIGHT * 256 - 1) // Maximum Y coordinate
struct Grain {
  int16_t  x,  y; // Position
  int16_t vx, vy; // Velocity
} grain[N_GRAINS];

uint32_t        prevTime   = 0;      // Used for frames-per-second throttle
uint8_t         img[WIDTH * HEIGHT]; // Internal 'map' of pixels


LSM303 accel;

void setup()
{
  uint8_t i, j, bytes;

  Serial.begin(115200);

  Wire.begin();
  accel.init();
  accel.enableDefault();    // +/- 2g range
  accel.writeAccReg(accel.CTRL_REG4_A, 0x00);   // 0x10 will change to +/- 4g range, 0x11 is +/- 8g -  but I like more sensitive
  
  LedSign::Init();

  memset(img, 0, sizeof(img)); // Clear the img[] array
  for(i=0; i<N_GRAINS; i++) {  // For each sand grain...
    do {
      grain[i].x = random(WIDTH  * 256); // Assign random position within
      grain[i].y = random(HEIGHT * 256); // the 'grain' coordinate space
      // Check if corresponding pixel position is already occupied...
      for(j=0; (j<i) && (((grain[i].x / 256) != (grain[j].x / 256)) ||
                         ((grain[i].y / 256) != (grain[j].y / 256))); j++);
    } while(j < i); // Keep retrying until a clear spot is found
    img[(grain[i].y / 256) * WIDTH + (grain[i].x / 256)] = 1; // Mark it
    grain[i].vx = grain[i].vy = 0; // Initial velocity is zero
  }
  //Serial.print("Setup done...grains:\n\r");
  //for(i=0; i<N_GRAINS; i++) {
  //   Serial.print(grain[i].x);
  //   Serial.print(" ");
  //   Serial.print(grain[i].y);
  //   Serial.print("\n\r");
  // }

}

void loop()
{
  uint32_t t;
  //Serial.print("Wait...\n\r");
  while(((t = micros()) - prevTime) < (1000000L / MAX_FPS));
  prevTime = t;
  //Serial.print("Done wait...\n\r");

  //Serial.print("Reading accel...\n\r");
  accel.readAcc();
  int16_t ax =  (accel.a.y >> 4) / 8,      // Transform accelerometer axes
          ay =  (accel.a.x >> 4) / 8,      // to grain coordinate space
          az = abs(accel.a.z >> 4) / 64; // Random motion factor
  az = (az >= 3) ? 1 : 4 - az;      // Clip & invert
  ax -= az;                         // Subtract motion factor from X, Y
  ay -= az;
  int16_t az2 = az * 2 + 1;         // Range of random motion to add back in

  //Serial.print("Accel read: ");
  //Serial.print(ax);
  //Serial.print(" ");
  //Serial.print(ay);
  //Serial.print(" ");
  //Serial.print(az);
  //Serial.print("\n\r");

  // ...and apply 2D accel vector to grain velocities...
  int32_t v2; // Velocity squared
  float   v;  // Absolute velocity
  for(int i=0; i<N_GRAINS; i++) {
    grain[i].vx += ax + random(az2); // A little randomness makes
    grain[i].vy += ay + random(az2); // tall stacks topple better!
    // Terminal velocity (in any direction) is 256 units -- equal to
    // 1 pixel -- which keeps moving grains from passing through each other
    // and other such mayhem.  Though it takes some extra math, velocity is
    // clipped as a 2D vector (not separately-limited X & Y) so that
    // diagonal movement isn't faster
    v2 = (int32_t)grain[i].vx*grain[i].vx+(int32_t)grain[i].vy*grain[i].vy;
    if(v2 > 65536) { // If v^2 > 65536, then v > 256
      v = sqrt((float)v2); // Velocity vector magnitude
      grain[i].vx = (int)(256.0*(float)grain[i].vx/v); // Maintain heading
      grain[i].vy = (int)(256.0*(float)grain[i].vy/v); // Limit magnitude
    }
  }

  //Serial.print("2d vector applied, updating position...\n\r");
  // ...then update position of each grain, one at a time, checking for
  // collisions and having them react.  This really seems like it shouldn't
  // work, as only one grain is considered at a time while the rest are
  // regarded as stationary.  Yet this naive algorithm, taking many not-
  // technically-quite-correct steps, and repeated quickly enough,
  // visually integrates into something that somewhat resembles physics.
  // (I'd initially tried implementing this as a bunch of concurrent and
  // "realistic" elastic collisions among circular grains, but the
  // calculations and volument of code quickly got out of hand for both
  // the tiny 8-bit AVR microcontroller and my tiny dinosaur brain.)

  uint8_t        i, bytes, oldidx, newidx, delta;
  int16_t        newx, newy;

  for(i=0; i<N_GRAINS; i++) {
    newx = grain[i].x + grain[i].vx; // New position in grain space
    newy = grain[i].y + grain[i].vy;
    if(newx > MAX_X) {               // If grain would go out of bounds
      newx         = MAX_X;          // keep it inside, and
      grain[i].vx /= -2;             // give a slight bounce off the wall
    } else if(newx < 0) {
      newx         = 0;
      grain[i].vx /= -2;
    }
    if(newy > MAX_Y) {
      newy         = MAX_Y;
      grain[i].vy /= -2;
    } else if(newy < 0) {
      newy         = 0;
      grain[i].vy /= -2;
    }

    oldidx = (grain[i].y/256) * WIDTH + (grain[i].x/256); // Prior pixel #
    newidx = (newy      /256) * WIDTH + (newx      /256); // New pixel #
    if((oldidx != newidx) && // If grain is moving to a new pixel...
        img[newidx]) {       // but if that pixel is already occupied...
      delta = abs(newidx - oldidx); // What direction when blocked?
      if(delta == 1) {            // 1 pixel left or right)
        newx         = grain[i].x;  // Cancel X motion
        grain[i].vx /= -2;          // and bounce X velocity (Y is OK)
        newidx       = oldidx;      // No pixel change
      } else if(delta == WIDTH) { // 1 pixel up or down
        newy         = grain[i].y;  // Cancel Y motion
        grain[i].vy /= -2;          // and bounce Y velocity (X is OK)
        newidx       = oldidx;      // No pixel change
      } else { // Diagonal intersection is more tricky...
        // Try skidding along just one axis of motion if possible (start w/
        // faster axis).  Because we've already established that diagonal
        // (both-axis) motion is occurring, moving on either axis alone WILL
        // change the pixel index, no need to check that again.
        if((abs(grain[i].vx) - abs(grain[i].vy)) >= 0) { // X axis is faster
          newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
          if(!img[newidx]) { // That pixel's free!  Take it!  But...
            newy         = grain[i].y; // Cancel Y motion
            grain[i].vy /= -2;         // and bounce Y velocity
          } else { // X pixel is taken, so try Y...
            newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
            if(!img[newidx]) { // Pixel is free, take it, but first...
              newx         = grain[i].x; // Cancel X motion
              grain[i].vx /= -2;         // and bounce X velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;         // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx       = oldidx;     // Not moving
            }
          }
        } else { // Y axis is faster, start there
          newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
          if(!img[newidx]) { // Pixel's free!  Take it!  But...
            newx         = grain[i].x; // Cancel X motion
            grain[i].vy /= -2;         // and bounce X velocity
          } else { // Y pixel is taken, so try X...
            newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
            if(!img[newidx]) { // Pixel is free, take it, but first...
              newy         = grain[i].y; // Cancel Y motion
              grain[i].vy /= -2;         // and bounce Y velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;         // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx       = oldidx;     // Not moving
            }
          }
        }
      }
    }

    //Serial.print("Updating grain position...\n\r");
    grain[i].x  = newx; // Update grain position
    grain[i].y  = newy;
    img[oldidx] = 0;    // Clear old spot (might be same as new, that's OK)
    img[newidx] = 1;  // Set new spot
  }

  // Update pixel data in LED driver
  //Serial.print("Now updating pixels...\n\r");
  for(i=0; i<WIDTH*HEIGHT; i++) {
    LedSign::Set(i % WIDTH, i / WIDTH, img[i]);
  }


}

