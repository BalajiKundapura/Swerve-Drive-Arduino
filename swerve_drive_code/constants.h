#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace Constants {
    // Physical constants in cm
    // length and width is from center of rotation of the module 
    constexpr float length = 19.579191;
    constexpr float width = 19.579191;
    constexpr float wheelRadius = 2.54;

    // offsets (arbitrary)
    constexpr float FLangleOffset = 80;  
    constexpr float FRangleOffset = 100;    
    constexpr float BLangleOffset = -94;
    constexpr float BRangleOffset = 110;

    constexpr int maxSpeed = 30 * wheelRadius; // Increasing speed too much can cause slippage of the motor since the field is essentially controlling the motor blind (no encoder)

    constexpr int maxDriverVoltage = 6; // these two values directly correlate to the torque of the drive motors since its in
    constexpr int maxMotorVoltage = 6;  // open loop mode, but putting it too high can cause overheating

    constexpr int deadzone = 5;
}

#endif