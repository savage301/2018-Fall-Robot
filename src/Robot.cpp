/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>
#include <WPILib.h>
#include <Joystick.h>
#include <RobotMap.h>
#include <Talon.h>
#include <WPILib.h>
#include <ADXRS450_Gyro.h>

class Robot : public frc::TimedRobot {
public:


	frc::Joystick controller1{ Driver1 };
	frc::Talon LeftDrive{ LeftDrivePWM };

	frc::Talon RightDrive{ 	RightDrivePWM };
	frc::ADXRS450_Gyro gyro{};

	double Threshold(double in,double thres){
		double out = in;
		if (in>thres){
		out = thres;
		}
		if(in<-1*thres){
		out = -1*thres;
		}
		return out;
	}

	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
		LeftDrive.SetInverted(true);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select between different autonomous modes using the dashboard. The
	 * sendable chooser code works with the Java SmartDashboard. If you
	 * prefer the LabVIEW Dashboard, remove all of the chooser code and
	 * uncomment the GetString line to get the auto name from the text box
	 * below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */





	void AutonomousInit() override {
		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString("Auto Selector",
		//		 kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {}

	void TeleopPeriodic() {

			double leftin = controller1.GetRawAxis(1);
			double rightin = controller1.GetRawAxis(5);

							double left1 = leftin*leftin*leftin;
							double right1 = rightin*rightin*rightin;

			LeftDrive.Set(left1);
			RightDrive.Set(right1);

			double gyro_val = gyro.GetAngle();

		frc::SmartDashboard::PutString("Gyro", std::to_string(gyro_val));

		bool buttona = controller1.GetRawButton(1);
		bool buttonb = controller1.GetRawButton(2);
		if (buttona){
			gyro.Reset();
		}

		if (buttonb){
		double error = 0 - gyro_val;

		double kp = .1;
		double output = kp * error;
		LeftDrive.Set(-1*output);
		RightDrive.Set(output);

		}
		bool buttonx = controller1.GetRawButton(3);
		if (buttonx){
			double error = 0 - gyro_val;

					double kp = .1;
					double output = kp * error;
					leftin = Threshold(leftin,0.75);

					LeftDrive.Set(leftin-output);
					RightDrive.Set(leftin+output);

		}



	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
