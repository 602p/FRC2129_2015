
package org.usfirst.frc.team2129.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.Rect;
import com.ni.vision.VisionException;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	RobotDrive robotDrive;
	final int frontLeftChannel	= 3;
    final int rearLeftChannel	= 2;
    final int frontRightChannel	= 1;
    final int rearRightChannel	= 0;
    
    Joystick translateStick = new Joystick(0);
    Joystick rotateStick   = new Joystick(1);
    Joystick elevatorStick = new Joystick(2);
    Jaguar   elevatorMotor = new Jaguar(4);
    Jaguar   intakeMotor1  = new Jaguar(5);
    Jaguar   intakeMotor2  = new Jaguar(6);
    AnalogInput elevatorTopLimit = new AnalogInput(1);
    AnalogInput elevatorTopLimitLightSensor = new AnalogInput(0);
    
    double   elevatorPower = 0.4;
    double   elevatorDecentPower = 0.4;
    double   intakePower   = 0.4;
    double   elevatorBonus = 0.1;
    double   elevatorGoal  = 260;
    
    boolean  visionBroken  = false;
    
	int session;
	Image frame;
	CameraServer server;
	long counter = 0;
	Rect rect;
	
	public Robot(){
		
		try{
			frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
	
	        // the camera name (ex "cam0") can be found through the roborio web interface
	        session = NIVision.IMAQdxOpenCamera("cam0",
	                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
	        NIVision.IMAQdxConfigureGrab(session);
	        NIVision.IMAQdxStartAcquisition(session);
		}catch (VisionException vx){
			visionBroken=true;
		}


       robotDrive = new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
       robotDrive.setExpiration(0.1);
       
       
//	   robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);	// invert the left side motors
//	   robotDrive.setInvertedMotor(MotorType.kRearLeft, true);		// you may need to change or remove this to match your robot
	}

    @SuppressWarnings("deprecation")
	public void teleopPeriodic() {
		this.counter++;
		SmartDashboard.putNumber("Counter", this.counter);
		
		if (!visionBroken){
			try{
				NIVision.IMAQdxGrab(session, frame, 1);
				CameraServer.getInstance().setImage(frame);
			}catch (VisionException vx){}
    	}
		
		this.elevatorPower=SmartDashboard.getDouble("ElevatorPower", 0.4);
		SmartDashboard.putNumber("ElevatorPower", elevatorPower);
		
		this.elevatorDecentPower=SmartDashboard.getDouble("ElevatorDecentPower", 0.4);
		SmartDashboard.putNumber("ElevatorDecentPower", elevatorDecentPower);
		
		this.elevatorBonus=SmartDashboard.getDouble("ElevatorBonus", 0.1);
		SmartDashboard.putNumber("ElevatorBonus", elevatorBonus);
		
		this.elevatorGoal=SmartDashboard.getDouble("ElevatorGoal", 0.1);
		SmartDashboard.putNumber("ElevatorGoal", elevatorGoal);
		
		this.intakePower=SmartDashboard.getDouble("IntakePower", 0.4);
		SmartDashboard.putNumber("IntakePower", intakePower);
		
		if (elevatorStick.getRawButton(4)){
			intakeMotor1.set(intakePower);
			intakeMotor2.set(intakePower);
		}else if (elevatorStick.getRawButton(5)){
			intakeMotor1.set(-intakePower);
			intakeMotor2.set(-intakePower);
		}else{
			intakeMotor1.set(0);
			intakeMotor2.set(0);
		}
		
		SmartDashboard.putNumber("ETLS_L_Int", elevatorTopLimitLightSensor.getValue());
		double rawExponentComponent = (float)Math.pow( (float)elevatorTopLimitLightSensor.getValue()-elevatorGoal , 1.5f);
		SmartDashboard.putNumber("AutoRaise_rawExponentComponent", rawExponentComponent);
		double exponentComponent = rawExponentComponent / 22000;
		SmartDashboard.putNumber("AutoRaise_exponentComponent", exponentComponent);
		double modifiedExponentComponent = exponentComponent * elevatorPower;
		SmartDashboard.putNumber("AutoRaise_modifiedExponentComponent", modifiedExponentComponent);
		double finalElevatorValue = modifiedExponentComponent + elevatorBonus;
		SmartDashboard.putNumber("AutoRaise_finalValue", finalElevatorValue);
		
		if (elevatorStick.getRawButton(3)){
			elevatorMotor.set(finalElevatorValue);
		}else if (elevatorStick.getRawButton(2)){
			elevatorMotor.set(-this.elevatorDecentPower);
		}else{
			elevatorMotor.set(0);
		}
		
		
		
		robotDrive.mecanumDrive_Cartesian(translateStick.getX(), translateStick.getY(), rotateStick.getX(), 0);
		Timer.delay(0.005);
    }
}