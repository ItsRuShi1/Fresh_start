#include "Dkinematics.h"
#include "motion.h"  // Include the motion class for interpolation functions
#include <SCServo.h>  // Include the SCServo library for serial-based servo control
#include <ESP32Servo.h>  // Include the ESP32Servo library
SMS_STS st;
Servo solenoidValve;  // Servo object to control the solenoid valve
Servo motor;          // Servo object to control the motor
#include <algorithm> 
#include <AccelStepper.h>
// Define the UART pins and baud rate for servo communication
#define S_RXD 18
#define S_TXD 19
#define stepPin 35
#define dirPin 34
// Define the servo position range for ST servos
const int ST_MIN_POSITION = 0;
const int ST_MAX_POSITION = 4095;

AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, stepPin, dirPin);


// Calculate the bits per degree based on the defined range for ST servos
const float ST_SERVO_RESOLUTION = (ST_MAX_POSITION - ST_MIN_POSITION) / 360.0f;

// Define the angle range for each joint
const float j_1_min = -50.0f;
const float j_1_max = 50.0f;
const float j_2_min = -50.0f;
const float j_2_max = 50.0f;
const float j_3_min = -50.0f;
const float j_3_max = 50.0f;

// Define home positions for each motor (in degrees)
const float HOME_POSITIONS[3] = {0.0f, 0.0f, 0.0f};

// Define the maximum buffer size
const int MAX_BUFFER_SIZE = 2000;
float positionBuffer[MAX_BUFFER_SIZE][3]; // Buffer to store joint angles
Point interpolationPoints[2]; // Buffer to store two interpolation points
bool gripperBuffer[MAX_BUFFER_SIZE];       // Buffer to store gripper states (true = suck, false = release)
int bufferIndex = 0;                      // Current index in the buffer
bool interpolationMode = false;           // Flag to check if we are in interpolation mode
int interpolationIndex = 0;               // Index to track interpolation points

// Variables to store the last calculated angles (before saving)
float lastTheta1, lastTheta2, lastTheta3;
bool lastGripperState = false;  // Last gripper state (false = release, true = suck)

// Variables for loop run control
bool loopRunActive = false;

// Variables for stepper motor control
bool stepperRunning = false;
bool stepperDirection = true; // true = forward, false = backward
float stepperSpeed = 1000.0; // steps per second
float stepperAcceleration = 500.0; // steps per second squared
long stepperMaxPosition = 2000; // Maximum position for continuous motion
long stepperMinPosition = 0; // Minimum position for continuous motion

// Map angle to servo position for ST servos
int angleToSTPosition(float angle) {
    return round((angle + 165.0f) * (ST_MAX_POSITION - ST_MIN_POSITION) / 360.0f);
}



void setup() {
    Serial.begin(115200);
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    st.pSerial = &Serial1;
    Dkinematics.init();
    Motion.init();  //  Initialize the motion system
    Serial.println("Ready to receive commands...");

    solenoidValve.attach(14); // Attach the solenoid valve to pin 2
    motor.attach(4);        // AttachS the motor to pin 15
    solenoidValve.write(0);  // Initialize the solenoid valve to the closed position
    motor.write(0);          // Initialize the motor to the off position
    pinMode(33, OUTPUT);
    
    // Initialize stepper motor
    stepper.setMaxSpeed(stepperSpeed);
    stepper.setAcceleration(stepperAcceleration);
    stepper.setCurrentPosition(0);
    
    // Move to initial home positions
    moveToHomePositions();
}

void moveToHomePositions() {
    moveServosSynchronized(HOME_POSITIONS[0], HOME_POSITIONS[1], HOME_POSITIONS[2]);
    
    Angle homeAngle = {HOME_POSITIONS[0], HOME_POSITIONS[1], HOME_POSITIONS[2]};
    Point homePoint;
    if (Dkinematics.ForwardKinematicsCalculations(homeAngle, homePoint)) {
        Motion.Data.CurrentPoint = homePoint;
        Motion.Data.CurrentAngle = homeAngle;
    }

    solenoidValve.write(0);  // Release the gripper
    motor.write(0);          // Turn off the motor
    delay(1000);
}

// Function to move all servos synchronously
void moveServosSynchronized(float theta1, float theta2, float theta3) {
    Serial.println("Sending servo commands...");
    Serial.print("Theta1: "); Serial.println(theta1);
    Serial.print("Theta2: "); Serial.println(theta2);
    Serial.print("Theta3: "); Serial.println(theta3);

    // Convert angles to positions
    int targetPosition1 = angleToSTPosition(constrain(theta1, j_1_min, j_1_max));
    int targetPosition2 = angleToSTPosition(constrain(theta2, j_2_min, j_2_max));
    int targetPosition3 = angleToSTPosition(constrain(theta3, j_3_min, j_3_max));

    Serial.print("TargetPosition1: "); Serial.println(targetPosition1);
    Serial.print("TargetPosition2: "); Serial.println(targetPosition2);
    Serial.print("TargetPosition3: "); Serial.println(targetPosition3);

    u8 ID[3] = {1, 2, 3}; // Servo IDs
    s16 Position[3] = {targetPosition1, targetPosition2, targetPosition3};
    u16 Speed[3] = {1500, 1500, 1500};  // You can adjust the speed if necessary
    u8 ACC[3] = {100, 100, 100};        // Set acceleration for each servo

    // Send commands to all servos at once
    st.SyncWritePosEx(ID, 3, Position, Speed, ACC);
}

  
void controlGripper(bool suck) {
    if (suck) {
        digitalWrite(33,HIGH);
        solenoidValve.write(0);
        // if  // Suck (open the valve)
        motor.write(180);   
        Serial.println("Sucking");     // Turn on the motor
    } else {
        digitalWrite(33,LOW);
        solenoidValve.write(180);  // Release (close the valve)
        motor.write(0);    
        Serial.println("Releasing");        // Turn off the motor
    }
}

// Save current interpolation point to buffer
void saveInterpolationPoint(Point point) {
    if (interpolationIndex < 2) {
        interpolationPoints[interpolationIndex] = point;
        interpolationIndex++;
        Serial.print("Interpolation point saved: X = ");
        Serial.print(point.X);
        Serial.print(", Y = ");
        Serial.print(point.Y);
        Serial.print(", Z = ");
        Serial.println(point.Z);
    } else {
        Serial.println("Interpolation buffer full.");
    }
}

void runInterpolation() {
    if (interpolationIndex == 2) {
        // Ensure points are correctly saved and different
        Serial.println("Running interpolation...");
        Serial.print("Starting Point: X = ");
        Serial.print(interpolationPoints[0].X);
        Serial.print(", Y = ");
        Serial.print(interpolationPoints[0].Y);
        Serial.print(", Z = ");
        Serial.println(interpolationPoints[0].Z);

        Serial.print("Ending Point: X = ");
        Serial.print(interpolationPoints[1].X);
        Serial.print(", Y = ");
        Serial.print(interpolationPoints[1].Y);
        Serial.print(", Z = ");
        Serial.println(interpolationPoints[1].Z);

        // Assign current and desired points for interpolation
        Motion.Data.CurrentPoint = interpolationPoints[0];  // Set starting point
        Motion.Data.DesiredPoint = interpolationPoints[1];  // Set ending point

        bool success = Motion.LinearInterpolation();  // Call the LinearInterpolation function
        if (!success) {
            Serial.println("Linear Interpolation failed.");
        }
    } else {
        Serial.println("Please savFe two points for interpolation.");
    }
}




// Function to run circle interpolation
void runCircleInterpolation(float i, float j, bool clockwise) {
    Serial.println("Running circle interpolation...");

    // Print inputs for verification
    Serial.print("Center Offset i: "); Serial.println(i);
    Serial.print("Center Offset j: "); Serial.println(j);
    Serial.print("Clockwise: "); Serial.println(clockwise);

    // Attempt circular interpolation
    bool success = Motion.CircleInterpolation(i, j, clockwise);  // Use circle interpolation from Motion.cpp
    
    if (success) {
        Serial.println("Circle interpolation completed successfully.");
    } else {
        Serial.println("Circle interpolation failed.");
    }
}


// Function to run Bezier curve interpolation between saved points
// Function to run Bezier curve interpolation between two control points
void runBezierInterpolation(Point p1, Point p2) {
    Serial.println("Running Bezier curve interpolation...");

    // Print control points for debugging
    Serial.print("Control Point 1: X = ");
    Serial.print(p1.X); Serial.print(", Y = ");
    Serial.print(p1.Y); Serial.print(", Z = ");
    Serial.println(p1.Z);

    Serial.print("Control Point 2: X = ");
    Serial.print(p2.X); Serial.print(", Y = ");
    Serial.print(p2.Y); Serial.print(", Z = ");
    Serial.println(p2.Z);

    // Run the Bezier curve interpolation
    bool success = Motion.Bezier4PointInterpolation(p1, p2);

    if (success) {
        Serial.println("Bezier curve interpolation completed successfully.");
    } else {
        Serial.println("Bezier curve interpolation failed.");
    }
}


void loopRunInterpolation() {
    if (interpolationIndex == 2) {
        loopRunActive = true;
        while (loopRunActive) {
            Serial.println("Looping interpolation...");

            // Number of steps for smooth motion between the two points
            int steps = 500; 

            // Move from Point 1 to Point 2
            Serial.println("Moving from Point 1 to Point 2...");
            Motion.Data.CurrentPoint = interpolationPoints[0];
            Motion.Data.DesiredPoint = interpolationPoints[1];
            if (!Motion.LinearInterpolation()) {
                Serial.println("Linear Interpolation failed.");
                break;
            }

            // Check for stop command after moving to Point 2
            if (Serial.available() > 0) {
                String input = Serial.readStringUntil('\n');
                input.trim();
                if (input.startsWith("stop")) {
                    loopRunActive = false;
                    Serial.println("Loop run stopped.");
                    break;
                }
            }

            delay(100);  // Optional: add a small delay for smooth movement

            // Move from Point 2 back to Point 1
            Serial.println("Moving from Point 2 to Point 1...");
            Motion.Data.CurrentPoint = interpolationPoints[1];
            Motion.Data.DesiredPoint = interpolationPoints[0];
            if (!Motion.LinearInterpolation()) {
                Serial.println("Linear Interpolation failed.");
                break;
            }

            // Check for stop command after moving back to Point 1
            if (Serial.available() > 0) {
                String input = Serial.readStringUntil('\n');
                input.trim();
                if (input.startsWith("stop")) {
                    loopRunActive = false;
                    Serial.println("Loop run stopped.");
                    break;
                }
            }

            delay(100);  // Optional: add a small delay for smooth movement
        }
    } else {
        Serial.println("Please save two points for interpolation before running the loop.");
    }
}

void loopRunCircleInterpolation(float i, float j, bool clockwise) {
    loopRunActive = true;

    while (loopRunActive) {
        Serial.println("Looping circular interpolation...");

        // Perform the circular interpolation
        Serial.println("Performing circular interpolation...");
        if (!Motion.CircleInterpolation(i, j, clockwise)) {
            Serial.println("Circular Interpolation failed.");
            break;
        }

        // Check for stop command after completing the circle
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            if (input.startsWith("stop")) {
                loopRunActive = false;
                Serial.println("Loop run stopped.");
                break;
            }
        }

        delay(100);  // Optional: add a small delay for smoother looping
    }
}


void saveCurrentStateToBuffer(float theta1, float theta2, float theta3, bool gripperState) {
    if (bufferIndex < MAX_BUFFER_SIZE) {
        positionBuffer[bufferIndex][0] = theta1;
        positionBuffer[bufferIndex][1] = theta2;
        positionBuffer[bufferIndex][2] = theta3;
        gripperBuffer[bufferIndex] = gripperState;
        bufferIndex++;
        Serial.println("State saved to buffer.");
    } else {
        Serial.println("Buffer is full. Cannot save more states.");
    }
}

void runSavedPositions() {
    u8 ID[3] = {1, 2, 3}; // Servo IDs
    s16 Position[3];
    u16 Speed[3] = {1000, 1000, 1000}; // Speeds for each servo
    u8 ACC[3] = {100, 100, 100}; // Acceleration for each servo

    for (int i = 0; i < bufferIndex; i++) {
        // Convert angles to positions
        for (int j = 0; j < 3; j++) {
            Position[j] = angleToSTPosition(positionBuffer[i][j]);
        }

        // Send commands to all servos at once
        st.SyncWritePosEx(ID, 3, Position, Speed, ACC);

        // Control the gripper based on the saved state
        controlGripper(gripperBuffer[i]);

        delay(1000); // Adjust delay as needed for smoother and more precise movement

        // Check if "stop" command is received during execution
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            if (input.startsWith("stop")) {
                loopRunActive = false;
                Serial.println("Loop run stopped.");
                return;
            }
        }
    }
}

// Function to move the Delta robot in different directions
void moveToPosition(String direction) {
  // if 

  //   if (direction == "MOVE +X") {
  //       moveMotor(1, 100);  // Motor 1 forward
  //       moveMotor(2, -100); // Motor 2 backward
  //       moveMotor(3, 0);    // Motor 3 idle
  //   } 
  //   else if (direction == "MOVE -X") {
  //       moveMotor(1, -100);
  //       moveMotor(2, 100);
  //       moveMotor(3, 0);
  //   } 
  //   else if (direction == "MOVE +Y") {
  //       moveMotor(1, 100);
  //       moveMotor(2, 100);
  //       moveMotor(3, -100);
  //   } 
  //   else if (direction == "MOVE -Y") {
  //       moveMotor(1, -100);
  //       moveMotor(2, -100);
  //       moveMotor(3, 100);
  //   } 
  //   else if (direction == "MOVE +Z") {
  //       moveMotor(1, 100);
  //       moveMotor(2, 100);
  //       moveMotor(3, 100);
  //   } 
  //   else if (direction == "MOVE -Z") {
  //       moveMotor(1, -100);
  //       moveMotor(2, -100);
  //       moveMotor(3, -100);
  //   } 
}

void StepperFunctions() {
    if (stepperRunning) {
        // Check if stepper has reached the end of travel
        long currentPos = stepper.currentPosition();
        
        if (stepperDirection) {
            // Moving forward - continue in forward direction
            if (currentPos >= stepperMaxPosition) {
                // Reset to min position and continue forward (wrap around)
                stepper.setCurrentPosition(stepperMinPosition);
                stepper.moveTo(stepperMaxPosition);
                Serial.println("Stepper: Reached max position, wrapping to start");
            } else if (!stepper.isRunning()) {
                // Continue moving forward if not at max position
                stepper.moveTo(stepperMaxPosition);
            }
        } else {
            // Moving backward - continue in backward direction
            if (currentPos <= stepperMinPosition) {
                // Reset to max position and continue backward (wrap around)
                stepper.setCurrentPosition(stepperMaxPosition);
                stepper.moveTo(stepperMinPosition);
                Serial.println("Stepper: Reached min position, wrapping to end");
            } else if (!stepper.isRunning()) {
                // Continue moving backward if not at min position
                stepper.moveTo(stepperMinPosition);
            }
        }
        
        // Run the stepper motor
        stepper.run();
    }
}

void loop() {
    // Call stepper functions for continuous motion control
    StepperFunctions();
    
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Remove any trailing whitespace or newline characters

        if (input.startsWith("interpolation")) {
            interpolationMode = true;
            interpolationIndex = 0; // Reset interpolation index
            Serial.println("Interpolation mode activated. Move the robot and save points.");
        } 
        else if (interpolationMode && input.startsWith("save")) {
            input.remove(0, 4);  // Remove "save"
            input.trim();  
            input.replace("[", "");  
            input.replace("]", "");  

            Point point;
            sscanf(input.c_str(), "%f,%f,%f", &point.X, &point.Y, &point.Z);
            saveInterpolationPoint(point);
        } 
        else if (input.startsWith("run") && interpolationMode) {
            runInterpolation();
        } 
        else if (input.startsWith("circle")) {
            float i, j;
            int clockwise;
            sscanf(input.c_str(), "circle %f,%f,%d", &i, &j, &clockwise);
            Motion.Data.DesiredPoint = Motion.Data.CurrentPoint;
            runCircleInterpolation(i, j, clockwise);
        } 
        else if (input.startsWith("bezier")) {
            Point p1, p2, endPoint;
            int parsed = sscanf(input.c_str(), "bezier [%f,%f,%f],[%f,%f,%f],[%f,%f,%f]", 
                                &p1.X, &p1.Y, &p1.Z, &p2.X, &p2.Y, &p2.Z, &endPoint.X, &endPoint.Y, &endPoint.Z);
            if (parsed == 9) {
                Motion.Data.DesiredPoint = endPoint;
                runBezierInterpolation(p1, p2);
            } else {
                Serial.println("Invalid bezier format.");
            }
        } 
        else if (input.startsWith("looprun") && interpolationMode) {
            loopRunInterpolation();
        } 
        else if (input.startsWith("loopruncircle")) {
            float i, j;
            int clockwise;
            sscanf(input.c_str(), "loopruncircle %f,%f,%d", &i, &j, &clockwise);
            Motion.Data.DesiredPoint = Motion.Data.CurrentPoint;
            loopRunCircleInterpolation(i, j, clockwise);
        } 
        else if (input.startsWith("stop") && loopRunActive) {
            loopRunActive = false;
            Serial.println("Loop run stopped.");
        } 
        else if (input.startsWith("FK")) {
            interpolationMode = false;
            input.remove(0, 2);
            input.trim();
            input.replace("[", "");
            input.replace("]", "");

            Angle angleposition;
            sscanf(input.c_str(), "%f,%f,%f", &angleposition.Theta1, &angleposition.Theta2, &angleposition.Theta3);

            Point point;
            if (Dkinematics.ForwardKinematicsCalculations(angleposition, point)) {
                Serial.print("FK Result -> X: ");
                Serial.print(point.X);
                Serial.print(", Y: ");
                Serial.print(point.Y);
                Serial.print(", Z: ");
                Serial.println(point.Z);

                moveServosSynchronized(angleposition.Theta1, angleposition.Theta2, angleposition.Theta3);

                Motion.Data.CurrentPoint = point;
                Motion.Data.CurrentAngle = angleposition;

                lastTheta1 = angleposition.Theta1;
                lastTheta2 = angleposition.Theta2;
                lastTheta3 = angleposition.Theta3;
            } else {
                Serial.println("FK Calculation failed.");
            }
        } 
        else if (input.startsWith("IK")) {
            interpolationMode = false;
            input.remove(0, 2);
            input.trim();
            input.replace("[", "");
            input.replace("]", "");

            Point point;
            sscanf(input.c_str(), "%f,%f,%f", &point.X, &point.Y, &point.Z);

            Angle angleposition;
            if (Dkinematics.InverseKinematicsCalculations(point, angleposition)) {
                Serial.print("IK Result -> Theta1: ");
                Serial.print(angleposition.Theta1);
                Serial.print(", Theta2: ");
                Serial.print(angleposition.Theta2);
                Serial.print(", Theta3: ");
                Serial.println(angleposition.Theta3);

                moveServosSynchronized(angleposition.Theta1, angleposition.Theta2, angleposition.Theta3);

                Motion.Data.CurrentPoint = point;
                Motion.Data.CurrentAngle = angleposition;

                lastTheta1 = angleposition.Theta1;
                lastTheta2 = angleposition.Theta2;
                lastTheta3 = angleposition.Theta3;
            } else {
                Serial.println("IK Calculation failed.");
            }
        } 
        else if (input.startsWith("suck")) {
            controlGripper(true);
            lastGripperState = true;
        } 
        else if (input.startsWith("release")) {
            controlGripper(false);
            lastGripperState = false;
        } 
        else if (input.startsWith("move")) {
            moveToPosition(input);
        } 
        else if (input.startsWith("home")) {
            interpolationMode = false;
            moveToHomePositions();
        } 
        else if (input.startsWith("StartStepper")) {
            stepperRunning = true;
            stepper.setMaxSpeed(stepperSpeed);
            stepper.setAcceleration(stepperAcceleration);
            if (stepperDirection) {
                stepper.moveTo(stepperMaxPosition);
            } else {
                stepper.moveTo(stepperMinPosition);
            }
            Serial.println("Stepper motor started");
        }
        else if (input.startsWith("StopStepper")) {
            stepperRunning = false;
            stepper.stop();
            Serial.println("Stepper motor stopped");
        }
        else if (input.startsWith("SetSpeed")) {
            input.remove(0, 8); // Remove "SetSpeed"
            input.trim();
            float newSpeed = input.toFloat();
            if (newSpeed > 0 && newSpeed <= 2000) {
                stepperSpeed = newSpeed;
                stepper.setMaxSpeed(stepperSpeed);
                Serial.print("Stepper speed set to: ");
                Serial.println(stepperSpeed);
            } else {
                Serial.println("Invalid speed. Please use a value between 1 and 2000 steps/second");
            }
        }
        else if (input.startsWith("SetAcceleration")) {
            input.remove(0, 15); // Remove "SetAcceleration"
            input.trim();
            float newAccel = input.toFloat();
            if (newAccel > 0 && newAccel <= 2000) {
                stepperAcceleration = newAccel;
                stepper.setAcceleration(stepperAcceleration);
                Serial.print("Stepper acceleration set to: ");
                Serial.println(stepperAcceleration);
            } else {
                Serial.println("Invalid acceleration. Please use a value between 1 and 2000 steps/second²");
            }
        }
        else if (input.startsWith("ChangeDirection")) {
            stepperDirection = !stepperDirection;
            if (stepperRunning) {
                if (stepperDirection) {
                    stepper.moveTo(stepperMaxPosition);
                } else {
                    stepper.moveTo(stepperMinPosition);
                }
            }
            Serial.print("Stepper direction changed to: ");
            Serial.println(stepperDirection ? "Forward" : "Backward");
        }
        else {
            Serial.println("Invalid command. Use interpolation, save [x,y,z], run, circle, bezier, looprun, FK [theta1,theta2,theta3], IK [x,y,z], suck, release, home, StartStepper, StopStepper, SetSpeed <value>, SetAcceleration <value>, or ChangeDirection.");
        }
    }
}