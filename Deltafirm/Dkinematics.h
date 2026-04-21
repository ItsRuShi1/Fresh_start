#include "math.h"
#ifndef _DKINEMATICS_h
#define _DKINEMATICS_h

#if defined(ARDUINO) && ARDUINO >= 100
    // #include "arduino.h"
#else
    #include "WProgram.h"
#endif

using namespace std;

// Define the constants that were originally in Constants.h
const double RD_F = 250.0;  // Replace with the actual value from Constants.h
const double RD_E = 100.0;  // Replace with the actual value from Constants.h
const double RD_RF = 145.0; // Replace with the actual value from Constants.h
const double RD_RE = 340.0 ; // Replace with the actual value from Constants.h

// Trigonometric constants
const double tan30 = 1 / sqrt(3);
const double tan30x05 = 0.5 / sqrt(3);
const double tan60 = sqrt(3);
const double sin30 = 0.5;
const double cos30 = sqrt(3) / 2;
const double cos120 = -0.5;
const double sin120 = sqrt(3) / 2;

// Define the Angle structure
struct Angle {
    float Theta1;
    float Theta2;
    float Theta3;
};

// Define the Point structure
struct Point {
    float X;
    float Y;
    float Z;
};

class DkinematicsClass
{
 protected:

 public:
    void init();
    bool ForwardKinematicsCalculations(Angle angleposition, Point &point);
    bool InverseKinematicsCalculations(Point point, Angle &angleposition);

private:
    bool AngleThetaCalculations(float x0, float y0, float z0, float &theta);
    float _y0_;
    float _y1_;
    float RD_RF_Pow2;
    float RD_RE_Pow2;
};

extern DkinematicsClass Dkinematics; // Declaration only

#endif
