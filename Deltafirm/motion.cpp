#include "motion.h"


extern void moveServosSynchronized(float theta1, float theta2, float theta3);
// Placeholder for Data structure
struct DataClass {
    Point CurrentPoint;
    Point DesiredPoint;
    Angle CurrentAngle;
    float MMPerLinearSegment = 1.0f;  // Set according to your system
    float MMPerArcSegment = 1.0f;     // Set according to your system
    float MOVING_AREA_LARGEST_DIAMETER = 200.0f; // Set this as needed
    float RD_W = 0.0f;
    float MOVING_AREA_Z = 100.0f;
    bool IsMoveWithTheAbsoluteCoordinates = true;  // Placeholder, adjust as needed
    float ZOffset = 0.0f;  // Placeholder for Z offset
    bool IsExecutedGcode = false;
};

DataClass Data;  // Declare the global Data object

// Define the missing constant
const int NUMBER_PER_BEZIER_SEGMENT = 10;  // Set this as appropriate for your setup

// Function to calculate the distance between two points
float CalDistance2Point(Point point1, Point point2) {
    float x_Offset = point1.X - point2.X;
    float y_Offset = point1.Y - point2.Y;
    float z_Offset = point1.Z - point2.Z;

    float distance = sqrt(pow(x_Offset, 2) + pow(y_Offset, 2) + pow(z_Offset, 2));

    if (distance < 0.2 && distance > -0.2) {
        distance = 0;
    }
    return distance;
}

// Function to linearly interpolate between two points
Point GetPointInLine(Point currentP, Point desiredP, float t) {
    Point buffer;
    buffer.X = currentP.X - ((currentP.X - desiredP.X) * t);
    buffer.Y = currentP.Y - ((currentP.Y - desiredP.Y) * t);
    buffer.Z = currentP.Z - ((currentP.Z - desiredP.Z) * t);
    return buffer;
}

// Function to get a point on a circle given the center (ox, oy), radius, and angle
Point GetPointInCircle(float ox, float oy, float radius, float angle) {
    Point buffer;
    buffer.X = ox + radius * cosf(angle);
    buffer.Y = oy + radius * sinf(angle);
    buffer.Z = Data.CurrentPoint.Z;
    return buffer;
}

// Function implementations

void MotionClass::init() {
    Serial.println("Initializing Motion...");

    // Initialize Data structure
    Data.CurrentPoint = {0.0, 0.0, 0.0};
    Data.DesiredPoint = {0.0, 0.0, 0.0};
    Data.MMPerLinearSegment = 1.0;  // Default linear segment value
    Data.MMPerArcSegment = 1.0;     // Default arc segment value
    Data.CurrentAngle = {0.0, 0.0, 0.0};

    // Other initialization steps
}


bool MotionClass::LinearInterpolation() {
    Serial.println("Starting Linear Interpolation...");  // Debugging line

    float distance2Point = CalDistance2Point(Data.CurrentPoint, Data.DesiredPoint);
    if (distance2Point == 0) {
        Serial.println("No distance between points, aborting.");  // Debugging line
        return false;
    }

    Angle angle_;
    Dkinematics.InverseKinematicsCalculations(Data.DesiredPoint, angle_);  // IK for the desired point
    Serial.print("Desired Angle1: "); Serial.println(angle_.Theta1);  // Debugging line

    uint16_t NumberSegment = floorf(distance2Point / Data.MMPerLinearSegment);
    if (NumberSegment < 1) NumberSegment = 1;

    float mm_per_seg = distance2Point / NumberSegment;
    Angle lastAngle = Data.CurrentAngle;
    Angle currentAngle;

    for (uint16_t i = 1; i <= NumberSegment; i++) {
        float tbuffer = (float)i / NumberSegment;
        Point pointBuffer = GetPointInLine(Data.CurrentPoint, Data.DesiredPoint, tbuffer);
        Dkinematics.InverseKinematicsCalculations(pointBuffer, currentAngle);  // IK for interpolation points

        Serial.print("Moving to point "); Serial.println(i);  // Debugging line
        Serial.print("Theta1: "); Serial.print(currentAngle.Theta1);  // Debugging line
        Serial.print(", Theta2: "); Serial.print(currentAngle.Theta2);  // Debugging line
        Serial.print(", Theta3: "); Serial.println(currentAngle.Theta3);  // Debugging line

        // Move servos to the calculated joint angles
        moveServosSynchronized(currentAngle.Theta1, currentAngle.Theta2, currentAngle.Theta3);

        UploadSegment(lastAngle, currentAngle, mm_per_seg, i - 1);
        lastAngle = currentAngle;
    }

    Data.CurrentPoint = Data.DesiredPoint;
    Data.CurrentAngle = currentAngle;

    return true;
}

bool MotionClass::CircleInterpolation(float i, float j, bool clockwise) {
    // Calculate the center of the circle
    float o_x = Data.CurrentPoint.X + i;
    float o_y = Data.CurrentPoint.Y + j;
    float radius = sqrt(pow(i, 2) + pow(j, 2));

    // Calculate current angle
    float angle_Current = acosf(-i / radius);
    if (j > 0) angle_Current = -angle_Current;

    // Calculate desired angle
    float angle_Desired = acosf((Data.DesiredPoint.X - o_x) / radius);
    if (Data.DesiredPoint.Y - o_y <= 0) angle_Desired = -angle_Desired;

    // Calculate angular travel
    float angular_travel = angle_Desired - angle_Current;
    
    // Check if it's a full circle request
    if (abs(Data.DesiredPoint.X - Data.CurrentPoint.X) < 0.1f && 
        abs(Data.DesiredPoint.Y - Data.CurrentPoint.Y) < 0.1f) {
        angular_travel = clockwise ? -2.0f * PI : 2.0f * PI;
    } else {
        if (angular_travel < 0) angular_travel += 2.0f * PI;
        if (clockwise) angular_travel -= 2.0f * PI;
    }

    // Total distance to travel along the circle
    float flat_mm = radius * abs(angular_travel);

    // Determine number of segments
    uint16_t NumberSegment = floorf(flat_mm / Data.MMPerLinearSegment);

    // Make sure there are enough segments for smooth motion
    if (NumberSegment < 7) {
        Serial.println("Too few segments, aborting.");
        return false;
    }

    // Calculate angle and distance per segment
    float theta_per_segment = angular_travel / NumberSegment;
    float mm_per_seg = flat_mm / NumberSegment;

    Angle lastAngle = Data.CurrentAngle;
    Angle currentAngle;

    // Move servos through each segment of the circle
    for (uint16_t i = 1; i < NumberSegment; i++) {
        // Calculate the intermediate point on the circle
        Point pointBuffer = GetPointInCircle(o_x, o_y, radius, angle_Current + (float)i * theta_per_segment);
        
        // Inverse kinematics for the current point
        if (!Dkinematics.InverseKinematicsCalculations(pointBuffer, currentAngle)) {
            Serial.println("IK Calculation failed, aborting.");
            return false;
        }

        // Debug: Print the calculated angles
        Serial.print("Moving to segment "); Serial.println(i);
        Serial.print("Theta1: "); Serial.println(currentAngle.Theta1);
        Serial.print("Theta2: "); Serial.println(currentAngle.Theta2);
        Serial.print("Theta3: "); Serial.println(currentAngle.Theta3);

        // Move the servos to the calculated joint angles
        moveServosSynchronized(currentAngle.Theta1, currentAngle.Theta2, currentAngle.Theta3);

        // Update the segment in the motion planner
        UploadSegment(lastAngle, currentAngle, mm_per_seg, i - 1);
        lastAngle = currentAngle;

        // Small delay for smooth movement
        delay(20); // Adjust as needed for smooth motion
    }

    // Move servos to the final desired point
    if (!Dkinematics.InverseKinematicsCalculations(Data.DesiredPoint, currentAngle)) {
        Serial.println("Final IK Calculation failed, aborting.");
        return false;
    }

    // Final segment movement
    moveServosSynchronized(currentAngle.Theta1, currentAngle.Theta2, currentAngle.Theta3);
    UploadSegment(lastAngle, currentAngle, mm_per_seg, NumberSegment - 1);

    // Update current state
    Data.CurrentPoint = Data.DesiredPoint;
    Data.CurrentAngle = currentAngle;

    return true;
}


bool MotionClass::Bezier4PointInterpolation(Point p1, Point p2) {
    // Number of segments for interpolation
    float tbuffer = 1.0 / NUMBER_PER_BEZIER_SEGMENT;
    
    // Initialize angles and points
    Angle lastAngle = Data.CurrentAngle;
    Angle currentAngle;
    Point lastPoint = Data.CurrentPoint;

    // Loop through segments and compute Bezier curve
    for (uint16_t i = 1; i <= NUMBER_PER_BEZIER_SEGMENT; i++) {
        // Compute intermediate Bezier points
        Point a = GetPointInLine(Data.CurrentPoint, p1, i * tbuffer);
        Point b = GetPointInLine(p1, p2, i * tbuffer);
        Point c = GetPointInLine(p2, Data.DesiredPoint, i * tbuffer);

        Point d = GetPointInLine(a, b, i * tbuffer);
        Point e = GetPointInLine(b, c, i * tbuffer);

        // Final interpolated point on the Bezier curve
        Point f = GetPointInLine(d, e, i * tbuffer);

        // Perform inverse kinematics for the current point
        if (!Dkinematics.InverseKinematicsCalculations(f, currentAngle)) {
            Serial.println("IK Calculation failed for Bezier interpolation");
            return false;
        }

        // Calculate distance between the points for each segment
        float distanceBuffer = CalDistance2Point(f, lastPoint);

        // Upload segment to motion planner
        UploadSegment(lastAngle, currentAngle, distanceBuffer, i - 1);

        // Move servos to the calculated joint angles
        moveServosSynchronized(currentAngle.Theta1, currentAngle.Theta2, currentAngle.Theta3);

        // Update last point and angle for the next iteration
        lastPoint = f;
        lastAngle = currentAngle;

        // Debugging - Print current segment information
        Serial.print("Bezier Segment: ");
        Serial.print(i);
        Serial.print(" | Theta1: ");
        Serial.print(currentAngle.Theta1);
        Serial.print(", Theta2: ");
        Serial.print(currentAngle.Theta2);
        Serial.print(", Theta3: ");
        Serial.println(currentAngle.Theta3);

        // Small delay to smooth the motion, adjust if necessary
        delay(5); 
    }

    // Update final position and angle after completing the curve
    Data.CurrentPoint = Data.DesiredPoint;
    Data.CurrentAngle = currentAngle;

    return true;
}

void MotionClass::UploadSegment(Angle angle1, Angle angle2, float distance, uint8_t index) {
    // Just print the angles for now (you will handle actual servo movements in the main code)
    Serial.print("Segment: ");
    Serial.print("Theta1: "); Serial.print(angle2.Theta1);
    Serial.print(", Theta2: "); Serial.print(angle2.Theta2);
    Serial.print(", Theta3: "); Serial.println(angle2.Theta3);
}

void MotionClass::moveToPosition(float xPos, float yPos, float zPos) {
    Angle anglePosition;
    if (Dkinematics.InverseKinematicsCalculations({xPos, yPos, zPos}, anglePosition)) {
        // Just print for now
        Serial.print("Moving to position with angles: ");
        Serial.print(anglePosition.Theta1); Serial.print(", ");
        Serial.print(anglePosition.Theta2); Serial.print(", ");
        Serial.println(anglePosition.Theta3);
    } else {
        Serial.println("IK Calculation failed.");
    }
}

MotionClass Motion;  // Create a global instance of the Motion class
