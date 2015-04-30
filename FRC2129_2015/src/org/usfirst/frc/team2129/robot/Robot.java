
package org.usfirst.frc.team2129.robot;

import java.io.PrintWriter;
import java.io.StringWriter;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.Rect;
import com.ni.vision.VisionException;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
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
    AnalogInput elevatorTopLimitPot = new AnalogInput(0);
    
    BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
    
    double   elevatorPower = 0.4;
    double   elevatorDecentPower = 0.4;
    double   intakePower   = 0.4;
    double   elevatorBonus = 0.1;
    double   elevatorGoal  = 260;
    
    double lastElevatorValue = 0;
    
    boolean  visionBroken  = false;
    
	int session;
	Image frame;
	CameraServer server;
	long counter = 0;
	Rect rect;
	private double elevatorDivisor;
	private boolean fieldCentric;
	private boolean disableSafteyMod;
	private double reverseAscentThreshold;
	private boolean reverseAscentEnabled;
	//And then the great god ted came upon us and said "these are cheap, so are these, use lots of them..."
	public Robot(){
		try{
			frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
	        // the camera name (ex "cam0") can be found through the roborio web interface
	        this.session = NIVision.IMAQdxOpenCamera("cam0",
	                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
	        NIVision.IMAQdxConfigureGrab(session);
	        NIVision.IMAQdxStartAcquisition(session);
		}catch (VisionException vx){
			System.out.print(vx);
			SmartDashboard.putString("LastError_iinit", vx.toString());
			StringWriter sw = new StringWriter();
			PrintWriter pw = new PrintWriter(sw);
			vx.printStackTrace(pw);
			SmartDashboard.putString("LastErrorTrace_iinit", sw.toString());
			visionBroken=true;
		}
		
	   accelerometer.setRange(Range.k2G);
       robotDrive = new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
	}
	
	@SuppressWarnings("deprecation")
	public void teleopPeriodic() {
		this.counter++;
		SmartDashboard.putNumber("Counter", this.counter);
		
		if (!visionBroken){
			try{
				NIVision.IMAQdxGrab(session, frame, 1);
				CameraServer.getInstance().setImage(frame);
			}catch (VisionException vx){
				SmartDashboard.putString("LastError_each", vx.toString());
				StringWriter sw = new StringWriter();
				PrintWriter pw = new PrintWriter(sw);
				vx.printStackTrace(pw);
				SmartDashboard.putString("LastErrorTrace_each", sw.toString());
				visionBroken=true;
			}
    	}
		
		this.intakePower=SmartDashboard.getDouble("IntakePower", 0.4);
		SmartDashboard.putNumber("IntakePower", intakePower);		
		SmartDashboard.putNumber("ETLS_L_Int", elevatorTopLimitPot.getValue());
		
		this.elevatorGoal=SmartDashboard.getDouble("elevatorGoal", 800);
		SmartDashboard.putNumber("elevatorGoal", elevatorGoal);
		
		this.elevatorDivisor=SmartDashboard.getDouble("elevatorDivisor", 300);
		SmartDashboard.putNumber("elevatorDivisor", elevatorDivisor);
		
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
			//elevatorMotor.set(finalElevatorValue);
			elevatorMotor.set(elevatorStick.getY());
		}else if (elevatorStick.getRawButton(2)){
			//elevatorMotor.set(-this.elevatorDecentPower);
		}else{
			SmartDashboard.putBoolean("ElevatorOvveride", false);
		}
		SmartDashboard.putBoolean("VisionBroken", visionBroken);
		if (elevatorStick.getRawButton(11)){
			visionBroken=false;
			try{
				NIVision.IMAQdxCloseCamera(session);
			}catch (VisionException vx){}
			try{
				
				frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		        // the camera name (ex "cam0") can be found through the roborio web interface
		        session = NIVision.IMAQdxOpenCamera("cam0",
		                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		        NIVision.IMAQdxConfigureGrab(session);
		        NIVision.IMAQdxStartAcquisition(session);
			}catch (VisionException vx){
				SmartDashboard.putString("LastError_reinit", vx.toString());
				StringWriter sw = new StringWriter();
				PrintWriter pw = new PrintWriter(sw);
				vx.printStackTrace(pw);
				SmartDashboard.putString("LastErrorTrac_reinit", sw.toString());
				visionBroken=true;
			}
			
		}
		
		this.fieldCentric=SmartDashboard.getBoolean("fieldCentric", false);
		SmartDashboard.putBoolean("fieldCentric", fieldCentric);
		
		this.disableSafteyMod=SmartDashboard.getBoolean("disableSaftey", false);
		SmartDashboard.putBoolean("disableSaftey", disableSafteyMod);
		
		this.reverseAscentEnabled=SmartDashboard.getBoolean("reverseAscentEnabled", true);
		SmartDashboard.putBoolean("reverseAscentEnabled", reverseAscentEnabled);
		
		SmartDashboard.putNumber("elevatorFinal", -elevatorStick.getY()+safteyMod);
		
		this.reverseAscentThreshold=SmartDashboard.getDouble("reverseAscentThreshold", 30);
		SmartDashboard.putNumber("reverseAscentThreshold", reverseAscentThreshold);
		
		boolean rad=false;
		
		if ((elevatorTopLimitPot.getValue()<reverseAscentThreshold) && (-elevatorStick.getY() < 0)){
			rad = true;
		}
		
		SmartDashboard.putBoolean("reverseAscentDetected", rad);
		SmartDashboard.putBoolean("reverseAscentEnabled", reverseAscentEnabled);
		
		if (elevatorStick.getRawButton(3) || true){
			if (!rad || !reverseAscentEnabled){
				elevatorMotor.set(-elevatorStick.getY()+safteyMod);
				SmartDashboard.putBoolean("ElevatorEnabled", true);
			}
		}
		
		double modifiedGyroValue = 0;
		double accelReading = accelerometer.getX();
		SmartDashboard.putNumber("accelerometerReading", accelReading);
		
		if (fieldCentric){
			modifiedGyroValue = accelReading;
		}
		
		robotDrive.mecanumDrive_Cartesian(-translateStick.getX(), translateStick.getY(), rotateStick.getX(), modifiedGyroValue);
		Timer.delay(0.005);
    }

	@SuppressWarnings("deprecation")
	public void autonomousPeriodic(){
		double strafeValue = SmartDashboard.getDouble("autoStrafe", 0);
		SmartDashboard.putDouble("autoStrafe", strafeValue);
		
		double driveValue = SmartDashboard.getDouble("autoDrive", 0);
		SmartDashboard.putDouble("autoDrive", driveValue);
		
		double rotateValue = SmartDashboard.getDouble("autoRotate", 0);
		SmartDashboard.putDouble("autoRotate", rotateValue);
		
		robotDrive.mecanumDrive_Cartesian(strafeValue, driveValue, rotateValue, accelerometer.getX());
		Timer.delay(0.005);
	}
}
