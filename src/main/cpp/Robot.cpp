#include <iostream>
#include <Joystick.h>
#include <SampleRobot.h>
#include <Talon.h>
#include "ctre/Phoenix.h"
#include <Timer.h>
#include <AHRS.h>
#include <math.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <SmartDashboard/SendableChooser.h>
#include <fstream>
#include <string>



class Robot: public frc::SampleRobot {
bool tractionDown = false;
bool grablock = false;
bool gyroenabled = false;
bool isJoystick = true;
double PI = 3.14159;
bool IsMechanum = true;
float SminimumY = .5;
float SminimumX = .5;
float TargetMin = .25;
float TurnMax = .5;
float TurnMin = .27;
float sCurveFactor = .5;
int StartPosition = 0;
bool isAutonomoose=true;
int R_INTAKE_INVERSE = 1;
int L_INTAKE_INVERSE = -1;
double lastTime = 0;
double lastTimeP = 0;
bool recordingEnabled = true;

bool endGame = false;

int imageXsize = 320, imageYsize = 240;
std::vector<double> visionX;
std::vector<double> visionY;

bool target = true;
int Xdesired = 160, Ydesired = 196;
float xval, yval;
float xpos = 0, ypos = 0;

public:
	void RobotInit() override {
		chooser.AddDefault("Go forward", "GoForward");
		chooser.AddObject("Position 2 Switch", "Pos2Switch");
		chooser.AddObject("Position 3 Switch", "Pos3Switch");
		chooser.AddObject("Position 3 Scale", "Pos3Scale");
		chooser.AddObject("Position 1 Scale", "Pos1Scale");
		frc::SmartDashboard::PutData(&chooser);
	}
	float ABS(float input) {
		if (input<0) {
			input = input * -1;
		}
		return input;
	}
	void smtNUM(std::string text, double num){
		SmartDashboard::SmartDashboard::PutNumber(text, num);
	}
	void smtSTR(std::string text){
		SmartDashboard::SmartDashboard::PutString(text, text);
	}
	void smtBOOL(std::string text, bool blean){
		SmartDashboard::SmartDashboard::PutBoolean(text, blean);
	}
	void tankDrive(float RY, float LY){
		m_rf.Set(-LY);
		m_lf.Set(-LY);
		m_lr.Set(RY);
		m_rr.Set(RY);
	}
	double sCurve(float joyVal){
		return (((1-sCurveFactor)*pow(joyVal,3)) + (sCurveFactor*joyVal));
	}
	void CanMechanum(float RX, float RY, float LX, float robotangle){

		float mag = sqrt((RY*RY)+(RX*RX));
		float joyangle = 0;
		if (gyroenabled) {
			robotangle = (((PI)/180)*robotangle);
			while ((robotangle >= (2*PI)) or (robotangle < 0)){
				if (robotangle >= (2*PI)){
					robotangle = robotangle - (2*PI);
				}

				else if (robotangle < 0){
					robotangle = robotangle + (2*PI);
				}
			}
			float dif = 0;
			if ((RX>=0) && (RY>0)) { // Top Right
				joyangle = atan(RX/RY) + 0;
				dif = joyangle - robotangle;
			}
			else if ((RX>0) && (RY<=0)) { // Bottom Right
				joyangle = (atan((-RY)/RX)) + ((PI)/2);
				dif = joyangle - robotangle;
			}
			else if ((RX<=0) && (RY<0)) { // Bottom Left
				joyangle = atan((-RX)/(-RY)) + (PI);
				dif = joyangle - robotangle;
			}
			else if ((RX<0) && (RY>=0)) { // Top Left
				joyangle = atan(RY/(-RX)) + ((3*PI)/2);
				dif = joyangle - robotangle;
			}
			RY = mag * cos(dif);
			RX = mag * sin(dif);
		}

		mag = sqrt((RY*RY)+(RX*RX));

		float lfVal = RY + LX + RX;
		float lrVal = RY + LX - RX;
		float rfVal = -RY + LX + RX;
		float rrVal = -RY + LX - RX;

		float maxval = ABS(lfVal);
		if (ABS(lrVal)>maxval) {
			maxval = lrVal;
		}
		if (ABS(rfVal)>maxval) {
			maxval = rfVal;
		}
		if (ABS(rrVal)>maxval) {
			maxval = rrVal;
		}

		if (ABS(maxval) > 1){
			lfVal = ((lfVal / ABS(maxval))*mag);
			lrVal = ((lrVal / ABS(maxval))*mag);
			rfVal = ((rfVal / ABS(maxval))*mag);
			rrVal = ((rrVal / ABS(maxval))*mag);
		}

		smtNUM("lfVal", lfVal);
		smtNUM("lrVal", lrVal);
		smtNUM("rfVal", rfVal);
		smtNUM("rrVal", rrVal);

		m_lf.Set(lfVal);
		m_lr.Set(lrVal);
		m_rf.Set(rfVal);
		m_rr.Set(rrVal);
	}

	float TurnTo(int theta) {
		while (theta >= 360) {
			theta -= 360;
		}
		while (theta < 0) {
			theta += 360;
		}
		float robotangle = MotionTracker.GetAngle();
		while (robotangle >= 360) {
			robotangle -= 360;
		}
		while (robotangle < 0) {
			robotangle += 360;
		}
		float dif = theta - robotangle; // left is negative

		if (ABS(dif)<(3)) {
			return 0;
		}

		if (dif<=-180) {
			dif += 360;
		}
		if (dif>180) {
			dif -= 360;
		}

		float speed = ((dif/180)*(TurnMax-TurnMin));

		if (speed<0) {
			speed -= TurnMin;
		}
		else if (speed>0) {
			speed += TurnMin;
		}

		return speed;

	}

	void CAM() {

		target = false;

		visionX = entryX.GetDoubleArray(llvm::ArrayRef<double>());
		if (visionX.size()>0) {
			SmartDashboard::PutNumber("VisionX",visionX[0]);
			target = true;
		}

		visionY = entryY.GetDoubleArray(llvm::ArrayRef<double>());
		if (visionY.size()>0) {
			SmartDashboard::PutNumber("VisionY",visionY[0]);
			target = true;
		}
		smtBOOL("target", target);

	}

	int GetMaxYIndex(){

		int maxIndex = 0, maxY;

		if (target) {
			maxY = visionY[0];
			for (unsigned int i = 0; i < visionY.size(); i++){

				if (visionY[i] > maxY){ //change the less than if y is in wrong direction
					maxY = visionY[i];
					maxIndex = i;
				}

			}

		}
		else {
			maxIndex = -1;
		}

		return maxIndex;
	}

	void Locate(){

		int Tindex = GetMaxYIndex();

		if (target == true && Tindex != -1) {
			grabSol.Set(DoubleSolenoid::DoubleSolenoid::kForward);
			grablock = true;
			yval = visionY[Tindex];
			xval = visionX[Tindex];

			ypos = ((Ydesired - yval)/(imageYsize / 2)); //gives the position in terms of 1 to -1 which can also be used for proportional movement speed
			xpos = ((Xdesired - xval)/(imageXsize / 2));

			if (yval >= 170){
				ypos = 0;
			}
			if (ABS(xval-Xdesired)<=15){
				xpos = 0;
			}

			if (ypos < -.05){
				ypos -= TargetMin;
			}
			else if (ypos > .05){
				ypos += TargetMin;
			}
			else {
				ypos = 0;
			}

//			if (xpos < -.05){
//				xpos -= .1;
//			}
//			else if (xpos > .05){
//				xpos += TargetMin;
//			}
//			else{
//				xpos = 0;
//			}
			smtNUM("Xpos", xpos);
			smtNUM("Ypos", ypos);
			CanMechanum(xpos, ypos, 0, 0);

		}
		else {
			grabSol.Set(DoubleSolenoid::DoubleSolenoid::kReverse);
			smtBOOL("Located", false);
			//CanMechanum(0, 0, .5, 0);
			grablock = false;
		}


	}
	void SetGrabber(double val){
		m_rIntake.Set(val*R_INTAKE_INVERSE);
		m_lIntake.Set(val*L_INTAKE_INVERSE);
	}
	void Accessories() {
		m_lifter.Set(opstick.GetY());
		//smtNUM("GetY", xdrive.GetY());
		//smtNUM("GetYChannel", xdrive.GetYChannel());
		if (opstick.GetRawButton(5)){
			SetGrabber(-.95);
		}
		else if (opstick.GetThrottle() > .25)
			SetGrabber(opstick.GetThrottle());

		else if (opstick.GetRawButton(6)){
			SetGrabber(.95);
		}
		else  if (opstick.GetTwist() > .25)
			SetGrabber(-opstick.GetTwist());
		else SetGrabber(0);


		if (opstick.GetRawButton(1)){
			grabSol.Set(DoubleSolenoid::DoubleSolenoid::kForward);
		}
		else if (!grablock&&!endGame){
			grabSol.Set(DoubleSolenoid::DoubleSolenoid::kReverse);
		}
		if (opstick.GetRawButton(3)){
			endGame = true;
		}
		else if (opstick.GetRawButton(2)){
			endGame = false;
		}

		if (endGame){
			if (ABS(opstick.GetY()) > .2){
				brakeSol.Set(DoubleSolenoid::DoubleSolenoid::kForward);
			}
			else{
				brakeSol.Set(DoubleSolenoid::DoubleSolenoid::kReverse);
			}
			grabSol.Set(DoubleSolenoid::DoubleSolenoid::kForward);
		}
		else {
			brakeSol.Set(DoubleSolenoid::DoubleSolenoid::kForward);
		}
	}





	void OperatorControl() {
		GetAutoName();
//		xdrive.SetXChannel(4);
//		xdrive.SetYChannel(5);
//		xdrive.SetZChannel(0);
//		xdrive.SetThrottleChannel(1);
		//xdrive.SetTwistChannel(2);
		opstick.SetYChannel(1);
		opstick.SetThrottleChannel(2);
		opstick.SetTwistChannel(3);

		brakeSol.Set(DoubleSolenoid::DoubleSolenoid::kReverse);
		m_rf.Set(0);
		m_lf.Set(0);
		m_rr.Set(0);
		m_lr.Set(0);
		MatchTime.Start();
		lastTimeP = 0;



		t.Reset();

		//while(MotionTracker.IsCalibrating()){}
		Comp->SetClosedLoopControl(true);


		while (IsOperatorControl() && IsEnabled()) {

			Accessories();
			CAM();

			float rightX;
			float rightY;
			float leftX;
			float leftY;

			rightX = rstick.GetX();
			rightY = rstick.GetY();
			leftX = lstick.GetX();
			leftY = lstick.GetY();

			if (rstick.GetRawButton(3)){
				MotionTracker.Reset();
			}
			if (rstick.GetRawButton(2)){
				IsMechanum = true;
			}
			else if ( lstick.GetRawButton(2)){
				IsMechanum = false;
			}

			if (IsMechanum){
				if (rstick.GetRawButton(1)){
					Locate();
				}
				else {
					//butterflySol.Set(frc::DoubleSolenoid::kReverse);
					smtNUM("Joystick In Y", rightY);
					smtNUM("Joystick in X", rightX);
					smtNUM("Joystick in turn", leftX);
					CanMechanum(rightX, -rightY, (leftX/1.1), MotionTracker.GetAngle());
				}
			}
			else if (!IsMechanum){
				tankDrive(rightY,leftY);
				//butterflySol.Set(frc::DoubleSolenoid::kForward);
			}

//
			smtNUM("autotime", autoTime.Get());
			Wait(kUpdatePeriod);

		}
	}

	void GetAutoName(){
		std::string gameData;
		chosenAuto = chooser.GetSelected();
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if(gameData.length() > 0){
			if(gameData[0] == 'R'){
				if (chosenAuto[5] == 'w'){
					chosenAuto += "Right";
				}
			}
			else {
				if (chosenAuto[5] == 'w'){
					chosenAuto += "Left";
				}
			}
			if (gameData[1] == 'R'){
				if (chosenAuto[5] == 'c'){
					chosenAuto += "Right";
				}
			}
			else {
				if (chosenAuto[5] == 'c'){
					chosenAuto += "Left";
				}
			}
		}
		chosenAuto += ".csv";
	}
	void Autonomous(){

		isAutonomoose = true;
		gyroenabled = true;
		MotionTracker.Reset();
		bool isRightSwitch = false;
		bool isRightScale = false;
		m_lifter.Set(-.5);
		Wait(3.5);
		m_lifter.Set(0);
		grabSol.Set(DoubleSolenoid::DoubleSolenoid::kReverse);
		




		if (isRightSwitch&&StartPosition == 1){
			CanMechanum(-.3, .75, 0, MotionTracker.GetAngle());
			Wait(1.87);
			CanMechanum(0, 0, 0, 0);
		}
		else if (isRightSwitch&&StartPosition == 2){
			CanMechanum(.7, .95, 0, MotionTracker.GetAngle());
			SetGrabber(.95);
			Wait(.5);
			SetGrabber(0);
			Wait(2);

			Wait(.5);
			CanMechanum(0, 0, 0, 0);
			SetGrabber(-.95);
			Wait(1);
			SetGrabber(0);
		}
		else if (isRightSwitch&&StartPosition == 3){
			//Lift
			CanMechanum(0, .95, 0, MotionTracker.GetAngle());
			SetGrabber(.95);
			Wait(.5);
			SetGrabber(0);
			Wait(2);
			CanMechanum(0, .5, -.75, MotionTracker.GetAngle());
			Wait(1);
			CanMechanum(0, 0, 0, 0);
			SetGrabber(-.95);
			Wait(1);
			SetGrabber(0);
			//Shoot Cube
		}
		else if (!isRightSwitch&&StartPosition == 1){
			//Lift
			CanMechanum(0, .75, 0, MotionTracker.GetAngle());
			Wait(2);
			CanMechanum(0, 0, 0, 0);
			m_rIntake.Set(-.95);
			m_lIntake.Set(.95);
			Wait(1);
			m_rIntake.Set(0);
			m_lIntake.Set(0);
			//Shoot Cube
		}
		else if(!isRightSwitch&&StartPosition == 2){
//			int counterMax = 229;
//			int counter = 1;
//			t.Start();
//			while (IsAutonomous() && IsEnabled()){
//				//.007688
//
//
//				smtNUM("auto timememe", t.Get());
//				smtNUM("auto last time", lastTimeP);
//				smtNUM("counter auto", counter);
//				if (t.Get()-lastTimeP > .025){
//					counter++;
//					if (counter <= counterMax)
//						playAuto(std::to_string(counter), autocode.CubeLeft_Center);
//					else
//						break;
//					lastTimeP = t.Get();
//
//				}
////				timesum += t.Get();
////				smtNUM("timeind", t.Get());
////				smtNUM("timesum", timesum);
////				smtNUM("counter", counter);
////				smtNUM("averageTime", timesum/counter);
//			}
			CanMechanum(-.95, .85, 0, MotionTracker.GetAngle());
			SetGrabber(.95);
			Wait(.5);
			SetGrabber(0);
			Wait(2);
			CanMechanum(-.95, 0, 0, 0);
			Wait(.5);
			CanMechanum(0, 0, 0, 0);
			SetGrabber(-.95);
			Wait(1);
			SetGrabber(0);
		}
		else if(!isRightSwitch&&StartPosition == 3){
			CanMechanum(0, .75, 0, MotionTracker.GetAngle());
			Wait(2);
			CanMechanum(0, 0, 0, 0);
		}
		else if (isRightScale&&StartPosition == 5){
			m_lifter.Set(-.95);
			CanMechanum(0, .95, 0, MotionTracker.GetAngle());
			Wait(2);
			CanMechanum(0, 0, 0, 0);
			m_lifter.Set(0);
			SetGrabber(-.95);
			Wait(1);
			SetGrabber(0);
		}
		else if (!isRightScale&&StartPosition == 5){
			CanMechanum(0, .95, 0, MotionTracker.GetAngle());
			Wait(2);
			CanMechanum(0, 0, 0, 0);
		}

		else if (!isRightScale&& StartPosition == 6){
			m_lifter.Set(-.95);
			CanMechanum(0, .95, 0, MotionTracker.GetAngle());
			Wait(2);
			CanMechanum(0, 0, 0, 0);
			m_lifter.Set(0);
			SetGrabber(-.95);
			Wait(1);
			SetGrabber(0);
		}
		else if (isRightScale&&StartPosition == 6){
			CanMechanum(0, .95, 0, MotionTracker.GetAngle());
			Wait(2);
			CanMechanum(0, 0, 0, 0);
		}
		else {
			smtNUM("Zval", rstick.GetZ());
			SetGrabber(.95);
			m_lifter.Set(-.95);
			CanMechanum(0, .75, 0, MotionTracker.GetAngle());
			Wait(2.5);
			m_lifter.Set(0);
			SetGrabber(0);
			Wait(2);
			CanMechanum(0, 0, 0, 0);
		}



		gyroenabled = false;
		isAutonomoose = false;
	}

private:
	frc::Joystick rstick  { 0 };
	frc::Joystick lstick  { 1 };
	//frc::Joystick xdrive  { 2 };
	frc::Joystick opstick { 3 };
	WPI_TalonSRX m_rf { 1 };
	WPI_TalonSRX m_rr { 12 };
	WPI_TalonSRX m_lf { 0 };
	WPI_TalonSRX m_lr { 13 };
	WPI_TalonSRX m_lifter { 2 };
	WPI_TalonSRX m_rIntake { 14 };
	WPI_TalonSRX m_lIntake { 15 };
	AHRS MotionTracker {SPI::Port::kMXP}; //new motion tracker
	DoubleSolenoid grabSol {0, 1};
	DoubleSolenoid butterflySol {2, 3};
	DoubleSolenoid brakeSol {5, 4};
	Timer MatchTime;
	Timer autoTime;
	Timer t;
	frc::SendableChooser<std::string> chooser;
	std::string chosenAuto;


	nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table = inst.GetTable("GRIP/CubeReport1");
	nt::NetworkTableEntry entryX = table->GetEntry("centerX");
	nt::NetworkTableEntry entryY = table->GetEntry("centerY");

	Compressor *Comp = new Compressor (0);
	static constexpr double kUpdatePeriod = 0.005;


};
;
START_ROBOT_CLASS(Robot)
