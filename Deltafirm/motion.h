// Motion.h

#ifndef _MOTION_h
#define _MOTION_h

#if defined(ARDUINO) && ARDUINO >= 100
    // #include "arduino.h"
#else
    #include "WProgram.h"
#endif

#include "Dkinematics.h"  // Use the custom kinematics library
#include <SCServo.h>      // Use the SCServo library for serial-based servo control

// Define DataStruct for storing points
struct DataStruct {
    Point CurrentPoint;
    Point DesiredPoint;
    float MMPerLinearSegment;  // Linear interpolation segment length
    float MMPerArcSegment;     // Arc interpolation segment length
    Angle CurrentAngle;
};

// Declare the MotionClass
class MotionClass
{
public:

    DataStruct Data; 
    void init();  // Initialize motion settings
    bool LinearInterpolation();  // Linear interpolation logic
    bool CircleInterpolation(float i, float j, bool clockwise);  // Circle interpolation based on center points
    bool Bezier4PointInterpolation(Point p1, Point p2);  // Bezier interpolation for smooth non-linear movement

private:
    void UploadSegment(Angle angle1, Angle angle2, float distance, uint8_t index);  // Helper function to upload segments
    void moveToPosition(float xPos, float yPos, float zPos);  // Helper function to move to a specific position using IK
};

extern MotionClass Motion;  // Declare global instance of MotionClass

#endif
