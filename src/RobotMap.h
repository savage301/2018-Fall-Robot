/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <WPILib.h>



/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
*/

// For example to map the left and right motors, you could define the
// following variables to use with your drivetrain subsystem.
constexpr int Driver1 = 0;
constexpr int Driver2 = 1;

constexpr int LeftDrivePWM = 0;
constexpr int RightDrivePWM = 1;

constexpr int ClawMotorLeft1PWM = 3;
constexpr int ClawMotorRight1PWM = 4;


constexpr int ElevatorPWM = 2;

constexpr int PCM = 1;
constexpr int BrakePort1 = 0;
constexpr int BrakePort2 = 1;

constexpr int Ramp1Port1 = 2;
constexpr int Ramp1Port2 = 3;
constexpr int ShifterPort1 = 4;
constexpr int ShifterPort2 = 5;

constexpr int BoxlightgateDIO = 6;

constexpr int ElevatorEncoder1 = 4;
constexpr int ElevatorEncoder2 = 5;

constexpr int UltrasonicPort = 0;

constexpr int RampLeftPWM = 6;
constexpr int RampRightPWM = 7;





// If you are using multiple modules, make sure to define both the port
// number and the module. For example you with a rangefinder:
// constexpr int kRangeFinderPort = 1;
// constexpr int kRangeFinderModule = 1;
