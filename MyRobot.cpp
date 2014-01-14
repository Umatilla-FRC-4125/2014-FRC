#include "WPILib.h"
#include "Networktables\Networktable.h"
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
class RobotDemo : public IterativeRobot
{
	RobotDrive myRobot; // robot drive system
	Joystick DriveStick; // only joystick
	Solenoid Sol1;
	Solenoid Sol2;
	Relay c;
	
	 
public:
	RobotDemo():
		
		myRobot(1, 3, 2, 4),	// these must be initialized in the same order
		DriveStick(1),
		Sol1(1),
		Sol2(2),
		c(2)
		{	
		
		
	
	
		
		
		myRobot.SetExpiration(0.1);
		this->SetPeriod(00); 	//Set update period to sync with robot control packets (20ms nominal)
		DriveStick.SetAxisChannel(Joystick::kTwistAxis, 3);
		DriveStick.SetAxisChannel(Joystick::kXAxis, 4);
		DriveStick.SetAxisChannel(Joystick::kYAxis, 1);
	}
	
/**
 * Robot-wide initialization code should go here.
 * 
 * Use this method for default Robot-wide initialization which will
 * be called when the robot is first powered on.  It will be called exactly 1 time.
 */
void RobotDemo::RobotInit(){

	//Sol1.Set(false);
	//Sol2.Set(false);
	
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
}

/**
 * Periodic code for autonomous mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in autonomous mode.
 */
void RobotDemo::AutonomousPeriodic() {
}

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
	
	
	c.Set(Relay::kOff);
	myRobot.MecanumDrive_Cartesian(DriveStick.GetX(),DriveStick.GetY(),DriveStick.GetTwist());
	Sol1.Set(false);
	Sol2.Set(false);
	if(DriveStick.GetRawButton(1)){Sol1.Set(true);Sol2.Set(false);}
	if(DriveStick.GetRawButton(4)){Sol2.Set(true);Sol1.Set(false);}
	if(DriveStick.GetRawButton(2)){c.Set(Relay::kForward);}
	
	SmartDashboard::PutBoolean("A",DriveStick.GetRawButton(1));
	SmartDashboard::PutBoolean("button Y",DriveStick.GetRawButton(4));
	SmartDashboard::PutNumber("X",DriveStick.GetX());
	SmartDashboard::PutNumber("Y",DriveStick.GetY());
	
	SmartDashboard::PutNumber("Twist",DriveStick.GetTwist());
	
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

START_ROBOT_CLASS(RobotDemo);

