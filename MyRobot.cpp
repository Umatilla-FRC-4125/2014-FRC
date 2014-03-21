#include "WPILib.h"
#include "Networktables\Networktable.h"
#include "Hotcam.h"
/**
 * This is a demo program showing the use of the RobotBase class.
 * The IterativeRobot class is the base of a robot application that will automatically call your
 * Periodic methods for each packet based on the mode.
 * PWM channels
 * 1=frontLeftMotorChannel
 * 2=frontrightmotorChannel,
 * 3=rearleftMotorChannel
 * 4=rearRightMotorChannel;
 * DI/O
 * 2=Compressor Switch
 * Relay
 * 2= Compressor Relay
 */
class RobotDemo: public IterativeRobot {
	Talon FL;
	Talon FR;
	Talon RL;
	Talon RR;
	Talon Shooter1;
	Talon Lift;
	Talon rampOut;
	Relay Grab1;
	RobotDrive myRobot; // robot drive system
	Joystick DriveStick; //Joystick for Driving
	Joystick ShootStick; // Joystick for shooting
	Joystick ShootStick2;
	Encoder Auton1;
	HotcamAuton Check;
public:
	RobotDemo() :
				// these must be initialized in the same order
				FL(1), FR(2), RL(3), RR(4), Shooter1(5), Lift(8), rampOut(7),
				Grab1(1), myRobot(FL, RL, FR, RR), DriveStick(1),
				ShootStick(2), ShootStick2(3), Auton1(1, 2, true) {
		myRobot.SetExpiration(0.1);
		this->SetPeriod(00); //Set update period to sync with robot control packets (20ms nominal)
		DriveStick.SetAxisChannel(Joystick::kTwistAxis, 1);
		DriveStick.SetAxisChannel(Joystick::kXAxis, 4);
		DriveStick.SetAxisChannel(Joystick::kYAxis, 3);
		myRobot.SetInvertedMotor(myRobot.kRearRightMotor, true);
		myRobot.SetInvertedMotor(myRobot.kFrontRightMotor, true);

	}
	/**
	 * Robot-wide initialization code should go here.
	 *
	 * Use this method for default Robot-wide initialization which will
	 * be called when the robot is first powered on.  It will be called exactly 1 time.
	 */
	void RobotDemo::RobotInit() {
		Auton1.SetMaxPeriod(.1);
		Auton1.SetMinRate(10);
		Auton1.SetDistancePerPulse(.0040);
		Auton1.SetReverseDirection(true);
		Auton1.SetSamplesToAverage(7);
	}
	/**
	 * Initialization code for disabled mode should go here.
	 *
	 * Use this method for initialization code which will be called each time
	 * the robot enters disabled mode.
	 */
	void RobotDemo::DisabledInit() {
	}
	/**
	 * Periodic code for disabled mode should go here.
	 *
	 * Use this method for code which will be called periodically at a regular
	 * rate while the robot is in disabled mode.
	 */
	void RobotDemo::DisabledPeriodic() {
	}
	/**
	 * Initialization code for autonomous mode should go here.
	 *
	 * Use this method for initialization code which will be called each time
	 * the robot enters autonomous mode.
	 */
	void RobotDemo::AutonomousInit() {
		Auton1.Start();
		FL.SetSafetyEnabled(false);
		FR.SetSafetyEnabled(false);
		RL.SetSafetyEnabled(false);
		RR.SetSafetyEnabled(false);

	}
	/**
	 * Periodic code for autonomous mode should go here.
	 *
	 * Use this method for code which will be called periodically at a regular
	 * rate while the robot is in autonomous mode.
	 *
	 * Wheel Circumference = 28.27 in.
	 */
	void RobotDemo::AutonomousPeriodic() {

		if (Auton1.GetStopped() || Auton1.GetDistance() < 4.5) {
			myRobot.MecanumDrive_Cartesian(0, -.75, 0);
		}
		if (Auton1.GetDistance() >= 4.5) {
			Lift.Set(.2);
			myRobot.MecanumDrive_Cartesian(0, 0, 0);
			bool AutonCheck = Check.Auton();
			if (AutonCheck = true) {
				Shooter1.Set(-1);
				sleep(1);			
				Shooter1.Set(0);
				sleep(1);
				rampOut.Set(1);
				sleep(1);
				rampOut.Set(-1);
				sleep(1);
				rampOut.Set(0);
				Lift.Set(0);
				sleep(5);
			} else {
				sleep(3);
				Shooter1.Set(-1);
				sleep(1);
				Shooter1.Set(0);
				sleep(1);
				rampOut.Set(1);
				sleep(1);
				rampOut.Set(-1);
				sleep(1);
				rampOut.Set(0);
				Lift.Set(0);
			}


		}
		SmartDashboard::PutBoolean("Hot Goal", Check.Auton()); 
		SmartDashboard::PutNumber("auton1", Auton1.GetDistance());
		SmartDashboard::PutNumber("auton1rate", Auton1.GetRate());
		SmartDashboard::PutNumber("auton1Direction", Auton1.GetDirection());

	}
	;
	/**
	 * Initialization code for teleop mode should go here.
	 *
	 * Use this method for initialization code which will be called each time
	 * the robot enters teleop mode.
	 */
	void RobotDemo::TeleopInit() {
	}
	/**
	 * Periodic code for teleop mode should go here.
	 *
	 * Use this method for code which will be called periodically at a regular
	 * rate while the robot is in teleop mode.
	 */
	void RobotDemo::TeleopPeriodic() {
		//DRIVE
		Auton1.Start();
		float RoundX;
		float RoundTwist;
		if (DriveStick.GetX() < 0.10 && DriveStick.GetX() > -0.10) {
			RoundX = 0;
		} else {
			RoundX = DriveStick.GetX();
		}
		if (DriveStick.GetTwist() < 0.10 && DriveStick.GetTwist() > -0.10) {
			RoundTwist = 0;
		} else {
			RoundTwist = DriveStick.GetTwist();
		}
		myRobot.MecanumDrive_Cartesian(RoundX, DriveStick.GetY(), RoundTwist);
		//FIRE!!
		if (ShootStick.GetRawButton(1)) {
			Shooter1.Set(-1);
		} else if (ShootStick.GetRawButton(2)) {
			Shooter1.Set(.5);
		} else if (ShootStick.GetRawButton(3)) {
			Shooter1.Set(-0.5);//pass
		} else {
			Shooter1.Set(0);
		}
		//Pickup
		if (ShootStick.GetRawButton(4)) {
			Grab1.Set(Relay::kForward);
		} else if (ShootStick.GetRawButton(5)) {
			Grab1.Set(Relay::kReverse);
		} else {
			Grab1.Set(Relay::kOff);
		}
		//LiftArm
		if (ShootStick.GetRawButton(6)) {
			Lift.Set(.6);
		} else if (ShootStick.GetRawButton(7)) {
			Lift.Set(-.6);
		} else {
			Lift.Set(0);
		}
		//Rampout
		if (ShootStick.GetRawButton(8)) {
			rampOut.Set(1);
		} else if (ShootStick.GetRawButton(9)) {
			rampOut.Set(-1);
		} else {
			rampOut.Set(0);
		}
		//SmartShield
		SmartDashboard::PutBoolean("A", DriveStick.GetRawButton(1));
		SmartDashboard::PutBoolean("button Y", DriveStick.GetRawButton(4));
		SmartDashboard::PutNumber("X", DriveStick.GetX());
		SmartDashboard::PutNumber("Y", DriveStick.GetY());
		SmartDashboard::PutNumber("Twist", DriveStick.GetTwist());
		SmartDashboard::PutNumber("x-round", RoundX);
		SmartDashboard::PutNumber("Twist round", RoundTwist);
		SmartDashboard::PutNumber("auton1", Auton1.GetDistance());
		SmartDashboard::PutNumber("auton1rate", Auton1.GetRate());
		SmartDashboard::PutNumber("auton1Direction", Auton1.GetDirection());
	}
	/**
	 * Initialization code for test mode should go here.
	 *
	 * Use this method for initialization code which will be called each time
	 * the robot enters test mode.
	 */
	void RobotDemo::TestInit() {
	}
	/**
	 * Periodic code for test mode should go here.
	 *
	 * Use this method for code which will be called periodically at a regular
	 * rate while the robot is in test mode.
	 */
	void RobotDemo::TestPeriodic() {
	}
};
START_ROBOT_CLASS(RobotDemo)
;
