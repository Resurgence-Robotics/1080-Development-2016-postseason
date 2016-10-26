#include "WPILib.h"
#include "math.h"
#include <PID.h>
#define tau 6.2831

long Map(float x, float in_min, float in_max, float out_min, float out_max){// use this function to match two value's scales proportionally
	return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
class Robot: public SampleRobot
{
	ADXRS450_Gyro gyro;
	Joystick stick1;  //disco stick
	Joystick stick2;
	Joystick stick3;
	CANTalon left1;
	CANTalon left2;
	CANTalon right1;
	CANTalon right2;
	CANTalon intake_arm;
	CANTalon lift1;
	CANTalon kittenapult1; //hell yeah B)
	CANTalon kittenapult2;
	DoubleSolenoid *shooter;
	AnalogInput intake_pot, lift_pot, kittenapult_pot, mode_pot;
	DigitalInput IntakeEye, LiftSwitch, auto_switch;
	Relay *intake;
	Encoder *Lenc;
	Encoder *Renc;
	Encoder *kenc;





public:
	Robot() :

			stick1(0), // in order as they are declared above. objects with pointers below.
			stick2(1),
			stick3(2),
			left1(0),
			left2(1),
			right1(14),
			right2(15),
			intake_arm(3),
			lift1(2),
			kittenapult1(13),
			kittenapult2(12),
			intake_pot(0),
			lift_pot(1),
			kittenapult_pot(2),
			mode_pot(3),
			IntakeEye(0),
			LiftSwitch(1),
			auto_switch(9)



	{
		CameraServer::GetInstance()->SetQuality(50);
				//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam1");
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		shooter = new DoubleSolenoid(0,1);
		intake = new Relay(1);
		Lenc = new Encoder(4,5, true, Encoder::EncodingType::k4X);
		Renc = new Encoder(2,3, true, Encoder::EncodingType::k4X);
		kenc = new Encoder(6, 7, true, Encoder::EncodingType::k4X);

	}
	void Drive(float distance)
	{
		float wheel_radius =3;
		float wheel_circumference = 2*M_PI*wheel_radius;
		int PPR = 1440;
		float enc_in = PPR/wheel_circumference;
		float Target = distance*enc_in;
		Lenc->Reset();
		Renc->Reset();
		printf("\n Renc: %i", Renc->Get());
		if(distance > 0)//what direction are we driving
		{
			while(Renc->Get()<Target)//while we haven't reached target
			{
				// drive forward
				left1.Set(0.5);
				left2.Set(0.5);
				right1.Set(0.5);
				right2.Set(0.5);
				printf("\n Renc: %i", Renc->Get()); // printing a response to the rio-log
				Wait(0.1);//wait to allow code time to execute
			}
			left1.Set(0.0);
			left2.Set(0.0);
			right1.Set(0.0);
			right2.Set(0.0);
		}
		if(distance < 0) //what direction are we driving
		{
			while(Renc->Get()>Target)//while we haven't reached target
			{
				// drive backwards
				left1.Set(-0.5);
				left2.Set(-0.5);
				right1.Set(-0.5);
				right2.Set(-0.5);
				printf("\n Renc: %i", Renc->Get()); // printing a response to the rio-log
				Wait(0.1); // wait to allow code to execute
			}
			left1.Set(0.0); // stahp
			left2.Set(0.0);
			right1.Set(0.0);
			right2.Set(0.0);

		}
	}
	void DriveFRC(float outputMagnitude, float curve)
	{
	float leftOutput, rightOutput;
	float m_sensitivity = 0.5;
	if (curve < 0)
	{
	   float value = log(-curve);
	   float ratio = (value - m_sensitivity)/(value + m_sensitivity);
	   if (ratio == 0) ratio =.0000000001;
	  leftOutput = outputMagnitude / ratio;
	  rightOutput = outputMagnitude;
	}
	else if (curve > 0)
	{
	  float value = log(curve);
	  float ratio = (value - m_sensitivity)/(value + m_sensitivity);
	   if (ratio == 0) ratio =.0000000001;
	   leftOutput = outputMagnitude;
	   rightOutput = outputMagnitude / ratio;
	}
	else
	{
	  leftOutput = outputMagnitude;
	  rightOutput = outputMagnitude;
	}
	left1.Set(-1*leftOutput);
	left2.Set(-1*leftOutput);
	right1.Set(rightOutput);
	right2.Set(rightOutput);
	}
	void drivestright(float time, float speed)
	{
		gyro.Reset();
		Wait(1.0);
		float kp = 0.003;
		float TimeElapsed =0.0;
		while(TimeElapsed<time)
		{
			DriveFRC(speed, kp*gyro.GetAngle());
			Wait(0.2);
			TimeElapsed=TimeElapsed+0.2;
		}
		DriveFRC(0.0,0.0);
	}
	void drivestrightwithencoders(float target, float speed)
	{
		gyro.Reset();
		Renc->Reset();
		Wait(1.0);
		float kp = 0.003;
		int enc =0;
			if(target>0)//forward incrementing positive
			{
				while(target>enc&&IsAutonomous())
				{

				enc=-1*Renc->Get();

				printf("\n enc:%i",enc);
				DriveFRC(speed, kp*gyro.GetAngle());
				Wait(0.01);
				}
			}
			if (target<0)//reverse incrementing negative
			{
				while(target<enc&&IsAutonomous())
				{
				enc=Renc->Get();
				printf("\n enc:%i",enc);
				DriveFRC(speed, kp*-1*gyro.GetAngle());
				Wait(0.01);
				}
			}
		DriveFRC(0.0,0.0);
	}
	void Turn (float angle)
		{
		float wheel_circumference=2*M_PI*8;
		int PPR = 1440;
		float enc_in = PPR/wheel_circumference;
		float theta = angle*M_PI/180; //math
		int radius = 14;
		int arch = M_PI*radius*theta;
		float target = -1*arch*enc_in;
		printf("\n arch: %i", arch);
		printf("/n target: %f", target);
		Wait (3.0);
		Lenc->Reset();
		Renc->Reset();
		printf("\n Renc: %i", Renc->Get());  //\n is new line
		while (Renc->Get()>target)
			{
			left1.Set(0.5); //turns right
			left2.Set(0.5);
			right1.Set(-0.5);
			right2.Set(-0.5);
			printf("\n Renc: %i", Renc->Get());
			Wait(0.1);
			}
		left1.Set(0.0); //stahp
		left2.Set(0.0);
		right1.Set(0.0);
		right2.Set(0.0);

		while (Renc->Get()<target)
			{
			left1.Set(-0.5); //turns left
			left2.Set(-0.5);
			right1.Set(0.5);
			right2.Set(0.5);
			printf("\n Renc: %i", Renc->Get()); //|:T | :T
			Wait(0.1);
			}
		left1.Set(0.0); //stahp
		left2.Set(0.0);
		right1.Set(0.0);
		right2.Set(0.0);
		}
	void shoot()
	{
		//catapult cocked!
		//intake raised?
		//ball loaded!
		//intake lowered!?
		//shooter solenoid release!
	}
	void Autonomous()  //codin hard or hardly codin see?
	{

//		Drive(6);  //5ft basically cuz im tryna make a square
//		Wait(2.0);
//		Turn(45);
//		Wait(2.0);
//		Drive(6);
//		Wait(2.0);
//		Turn(45);
//		Wait(2.0);
//		Drive(6);
//		Wait(2.0);
//		Turn(45);
//		Wait(2.0);
//		Drive(6);
		//intakecontrol->SetSetpoint(1);
		printf("SW:%i\n", auto_switch.Get());

		int Auto_Sel=Map(mode_pot.GetVoltage(), 0, 5, 1, 12);
		printf("Mode:%i \n", Auto_Sel);
		if(auto_switch.Get())
		{

			if(Auto_Sel==1)// low passage 		// ARM DOWN
			{
				intake_arm.Set(-1.0);
				Wait(1);
				intake_arm.Set(0.0);
				drivestright(4, 0.4);
			}
			if(Auto_Sel==3)//mote 				// ARM UP HUMAN
			{
//				intake_arm.Set(-1.0);
//				Wait(1);
//				intake_arm.Set(0.0);
				drivestright(3.5, 0.8);
			}
			if(Auto_Sel==2)// rough terrain 	// ARM UP HUMAN
			{
//				intake_arm.Set(-1.0);
//				Wait(1);
//				intake_arm.Set(0.0);
				drivestright(2.5, 0.8);
			}
//			if(Auto_Sel==4)
//			{
//				intake_arm.Set(-1.0);
//				Wait(1);
//				intake_arm.Set(0.0);
//			}
//			intake_arm.Set(-1.0);
//			Wait(1);
//			intake_arm.Set(0.0);
//			drivestright(4, 0.4);//low passage

		}
		else
		{}
	//drivestright(2, 0.8);// for mote
	//drivestright(1.5, 0.8);// for rough terrain
	//		left1.Set(-0.6);
	//		left2.Set(-0.6);
	//		right1.Set(0.6);
	//		right2.Set(0.6);
	//		Wait(3.25);
	//		left1.Set(-0.0);
	//		left2.Set(-0.0);
	//		right1.Set(0.0);
	//		right2.Set(0.0);
	//		Shoot();  //define later
	//		Drive_To_Line();  //define l8r
	//		Drive(25.0);//SUCCESSSSSS 2/3/16 B)
	//		Drive(-25.0); //success 2/3/16 B)
	//		Turn (-90); //success 2/4/16 B)
	}
	void OperatorControl() //code in curly brackets conditions in parenthesis
	{

		while (IsOperatorControl() && IsEnabled())
		{
			//drivetrain //working
			left1.Set(-1*stick1.GetY());
			left2.Set(-1*stick1.GetY());
			printf("Y1:%f", stick1.GetY());
			right1.Set(stick2.GetY());
			right2.Set(stick2.GetY());

			//lift
			if(stick2.GetTrigger()&&LiftSwitch.Get()==false)//UP // "==" means comparative, "= means assignment"
			{
				lift1.Set(1.0);
				//lift2.Set(1.0);
				//turn motor on : -1.0 to 1.0

			}
			else if(stick2.GetRawButton(2))//DOWM
			{
				lift1.Set(-1.0);
				//lift2.Set(-1.0);
				//reverse motor
			}
			else
			{
				lift1.Set(0.0);
				//lift2.Set(0.0);
				//stop motor
			}

			//intake

			if(stick3.GetTrigger())//
			{
				//CameraServer::GetInstance()->StartAutomaticCapture("cam0");
				intake->Set(Relay::Value::kForward);

			}
			else
			{
				intake->Set(Relay::Value::kOff);
			}

			//intake arm
			//264down low goal load
			if(stick3.GetRawButton(2)&& (IntakeEye.Get()==1))//ball absent
			{
				intake->Set(Relay::Value::kReverse);
				//intakecontrol->SetSetpoint(0);
				intake_arm.Set(-0.5);
			}
			else if(stick3.GetRawButton(2)&& (IntakeEye.Get()==0))//ball there
			{
							//intake->Set(Relay::Value::kReverse);
							//intakecontrol->SetSetpoint(0);
							intake_arm.Set(-0.5);
			}
			else if(stick3.GetRawButton(4))
			{
				//intakecontrol->SetSetpoint(1.0);
				intake_arm.Set(0.5);
			}
			else //if(stick2.GetRawButton(6))
			{

				//intakecontrol->SetSetpoint(4.5);
				intake_arm.Set(0.0);
			}
			Wait(0.005);				// wait for a motor update time
		}
	}
	void Test()
	{
			kenc->Reset();
			int pre_fire =420;
			bool isRunning=false;
			bool isFired=false;
			float output;

		while(IsEnabled() &&IsTest())
		{
			printf("\n kenc: %i", kenc->GetRaw());
			if(stick3.GetRawButton(7)&&(isFired==false && isRunning==false))// fire catapult
			{
				shooter->Set(DoubleSolenoid::Value::kForward);
				kenc->Reset();
				isFired=true;
			}
			else if(isFired==true || stick3.GetRawButton(8))//cock catapult
			{
				isRunning=true;
				shooter->Set(DoubleSolenoid::Value::kReverse);//re-engage
				Wait(0.1);
				shooter->Set(DoubleSolenoid::Value::kOff);	  //our gearbox
				output=PID(pre_fire, kenc->GetRaw(), 0.003);
				printf("\n output: %f", output);
				kittenapult1.Set(output);
				kittenapult2.Set(output);
			}
			else if(output<0.1)//is ready to fire
			{
				isFired=false;
				isRunning=false;
			}

			output=PID(pre_fire, kenc->GetRaw(), 0.006);//Proportional only control.
			printf("\n output: %f", output);
			kittenapult1.Set(output);
			kittenapult2.Set(output);
			Wait(0.005);
		}//loop
	}
};

START_ROBOT_CLASS(Robot)
