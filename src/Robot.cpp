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
#include <Encoder.h>
#include "NetworkTables/NetworkTable.h"
#include <iostream>
#include <SerialPort.h>
#include <DigitalInput.h>

class Robot : public frc::TimedRobot {
public:


	frc::Joystick controller1{ Driver1 };
	frc::Talon LeftDrive{ LeftDrivePWM };

	frc::Talon RightDrive{ 	RightDrivePWM };
	frc::ADXRS450_Gyro gyro{};

	frc::SerialPort Laser{ 115200, SerialPort::Port::kUSB1};
	DigitalInput Arduino {0};
	frc::Encoder encoder {2, 3, false, Encoder::k4X};


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

		std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
		table->PutNumber("ledMode",1);
		table->PutNumber("pipeline",0);
		//Laser.Reset();
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

			bool buttonlb = controller1.GetRawButton(5);
			std::shared_ptr<NetworkTable> table =  NetworkTable::GetTable("limelight");
			table->PutNumber("ledMode",0);
			table->PutNumber("camMode",0);
			table->PutNumber("pipeline",0);

			float camera_x = table->GetNumber("tx",0);
			float camera_exist = table->GetNumber("tv",0);
			float image_size = table->GetNumber("ta",0);

			frc::SmartDashboard::PutString("tx Cam", std::to_string(camera_x));
			frc::SmartDashboard::PutString("tv Cam", std::to_string(camera_exist));
		frc::SmartDashboard::PutString("Gyro", std::to_string(gyro_val));

		double encoder_value = encoder.Get();
		frc::SmartDashboard::PutString("DB/String 2", std::to_string(encoder_value));

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

		bool buttony = controller1.GetRawButton(4);
		bool buttonrb = controller1.GetRawButton(6);

		if (buttony){
			encoder.Reset();

		}
		if (buttonrb){
			double distance = 10;
			double error = distance*(428/5) - encoder_value;

			double kp = -.0015;
			double output = kp * error;


			double error_g = 0 - gyro_val;

			double kp_g = .05;
			double output_g = kp_g * error_g;
			leftin = Threshold(output_g,0.75);
			output = Threshold(output,0.75);
			LeftDrive.Set(-1*leftin+output);
			RightDrive.Set(leftin+output);
		}



				if (buttonlb and camera_exist == 1){
					double error = 0 - camera_x;
					double error_size = image_size - 10.85;
							double kp_c = .025;
							double output = kp_c * error;
							leftin = Threshold(leftin,0.75);

							double k_image = .025;
							double output_image = k_image * error_size;
							output_image = Threshold(output_image,0.85);

							LeftDrive.Set(output_image+output);
							RightDrive.Set(output_image-output);

				}
				//Laser.Reset();
				bool valueout = Arduino.Get();
				frc::SmartDashboard::PutString("DB/String 7", std::to_string(valueout));




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
