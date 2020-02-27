/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  Joystick controller = new Joystick(0), Controller2 = new Joystick(1);

  //Button numbers for controls
  final int A = 2, B = 3, X = 1, Y = 4, leftBumper = 5, rightBumper = 6, leftTrigger = 7, rightTrigger = 8, back = 9, start = 10;

  //DriveBase Objects
  TalonSRX frontLeftRot, frontRightRot, rearLeftRot, rearRightRot;
  CANSparkMax frontLeft, frontRight, rearLeft, rearRight;

  SRF_Swerve_Module FLModule;
  SRF_Swerve_Module FRModule;
  SRF_Swerve_Module RLModule;
  SRF_Swerve_Module RRModule;
  SRF_Swerve_Drive driveBase;

  //Climbing motors
  TalonSRX winch1, winch2, hookLift;

  //Ball Motors
  //Full rotation = 1024 Counts
  TalonSRX carousel;
  VictorSPX intake, outtake;
  CANSparkMax shooter;

  CANPIDController shooterPID;
  CANEncoder shooterEncoder;

  AHRS navx;
  Block pixyBlock;
  PixyCam pixy = new PixyCam();
  I2C wire = new I2C(Port.kOnboard, 0x0);

  ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  //Pneumatics
  DoubleSolenoid arm, spinner, hood;
  //DoubleSolenoid arm = new DoubleSolenoid(0, 0);
  Compressor arnold;
  //Compressor arnold = new Compressor();
  boolean armDown = false; //if arm is up, armUp is true
  boolean hoodDown = false;
  boolean spinnerDown = false;

  final double drivekP = 16, drivekI = 0.00013, drivekD = 0;
  final double shootkP = 0.0005, shootkI = 0.00000027, shootkD = 0;
  final double carokP = 3.75, carokI = 0, carokD = 0;
  final double caroPoskP = 1.75, caroPoskI = 0.001, caroPoskD = 0;
   
  //Joystick values for swerve drive
  double x, y, w;

  double zeroYaw, gyroAngle;

  boolean testRotationController = true;

  Boolean letUpB = true, letUpX = true, letUpStart = true, letUpBack = true, letUpRBump;

  Double carouselSpeed, outtakeSpeed, shooterSpeed;
  Boolean carouselVelPID = true;
  int prevCarouselPos;
  Boolean slowMode = false, unjam = false;
  int carouselStartPos;
  //0 -17 20 17
  final int offSetFL = 0, offSetFR = 0, offSetRL = 0, offSetRR = 0;

  //Tim's Room
  boolean timStart = false;
  Timer tim = new Timer();

  UsbCamera cam;

  @Override
  public void robotInit() {
    //Driving initialization
    navx = new AHRS();

    frontLeft = new CANSparkMax(2, MotorType.kBrushless);
    frontRight = new CANSparkMax(8, MotorType.kBrushless);
    rearLeft = new CANSparkMax(4, MotorType.kBrushless);
    rearRight = new CANSparkMax(6, MotorType.kBrushless);

    frontLeftRot = new TalonSRX(1);
    frontRightRot = new TalonSRX(7);
    rearLeftRot = new TalonSRX(3);
    rearRightRot = new TalonSRX(5);

    FLModule = new SRF_Swerve_Module(1, 2, drivekP, drivekI, drivekD, offSetFL);
    FRModule = new SRF_Swerve_Module(7, 8, drivekP, drivekI, drivekD, offSetFR);
    RLModule = new SRF_Swerve_Module(3, 4, drivekP, drivekI, drivekD, offSetRL);
    RRModule = new SRF_Swerve_Module(5, 6, drivekP, drivekI, drivekD, offSetRR);

    driveBase = new SRF_Swerve_Drive(FLModule, FRModule, RLModule, RRModule, 25.0, 21.0, 32.65);

    //climbing initialization
    winch1 = new TalonSRX(14);
    winch2 = new TalonSRX(15);
    hookLift = new TalonSRX(13);

    //Ball Manipulating initialization
    carousel = new TalonSRX(12);
    carousel.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    carousel.config_kP(0, carokP);
    carousel.config_kI(0, carokI);
    carousel.config_kD(0, carokD);
    carousel.setNeutralMode(NeutralMode.Coast);
    
    intake = new VictorSPX(10);
    outtake = new VictorSPX(11);
    
    shooter = new CANSparkMax(16, MotorType.kBrushless);
    shooter.setIdleMode(IdleMode.kCoast);
    shooterPID = new CANPIDController(shooter);
    shooterPID.setP(shootkP);
    shooterPID.setI(shootkI);
    shooterPID.setD(shootkD);
    shooterEncoder = new CANEncoder(shooter);

    arnold = new Compressor(0);
    arnold.setClosedLoopControl(true);

    arm = new DoubleSolenoid(9,3,4);
    spinner = new DoubleSolenoid(9,2,5);
    hood = new DoubleSolenoid(9,1,6);

    cam = CameraServer.getInstance().startAutomaticCapture();
    //cam.setResolution(320, 240);
  }
 
  public void disabledPeriodic(){
    //SmartDashboard.putNumber("HookLift", hookLift.getSelectedSensorPosition());
    /*SmartDashboard.putNumber("Front Left Sensor", frontLeftRot.getSelectedSensorPosition());
    SmartDashboard.putNumber("Front Right Sensor", frontRightRot.getSelectedSensorPosition());
    SmartDashboard.putNumber("Rear Left Sensor", rearLeftRot.getSelectedSensorPosition());
    SmartDashboard.putNumber("Rear Right Sensor", rearRightRot.getSelectedSensorPosition());*/
    /*SmartDashboard.putNumber("gyro", navx.getAngle()%360);*/
  }

  @Override
  public void autonomousInit() {
    timStart = false;
    tim.stop();
    tim.reset();
    carouselStartPos = carousel.getSelectedSensorPosition();
    hood.set(Value.kForward);
    shooter.set(0);
  }

  @Override
  public void autonomousPeriodic() {
    if(carousel.getSelectedSensorPosition() < carouselStartPos + 1024) {
      shooterPID.setReference(3050, ControlType.kVelocity);

      if(shooterEncoder.getVelocity() > 3000){
        carousel.set(ControlMode.Velocity, -40);
        outtake.set(ControlMode.PercentOutput, -.6);
      } else {
        carousel.set(ControlMode.PercentOutput, 0);
        outtake.set(ControlMode.PercentOutput, 0);
      }
    } else {

      if(timStart == false) {
        tim.start();
        timStart = true;
        shooter.set(0);
        carousel.set(ControlMode.PercentOutput, 0);
        outtake.set(ControlMode.PercentOutput, 0);
        hood.set(Value.kReverse);
      }
      if(tim.get() < 1)
        driveBase.set(0, -.15, 0);
      else
        driveBase.set(0, 0, 0);
    }
    SmartDashboard.putNumber("tim", tim.get());
    SmartDashboard.putNumber("shoot velocity", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("caro velocity", carousel.getSelectedSensorVelocity());
  }

  @Override
  public void teleopInit() {
    zeroYaw = navx.getAngle() % 360;
    if(zeroYaw < 0)
      zeroYaw += 360;

    letUpRBump = true;
    timStart = false;
    prevCarouselPos = carousel.getSelectedSensorPosition();
    arm.set(Value.kForward);
    spinner.set(Value.kReverse);
    hood.set(Value.kReverse);
  }

  @Override
  public void teleopPeriodic() {
    if(carouselVelPID)
      carouselSpeed = 0.0;
    shooterSpeed = 0.0;
    outtakeSpeed = 0.0;

    x = controller.getRawAxis(0);
    y = controller.getRawAxis(1);
    w = controller.getRawAxis(2);
    gyroAngle = navx.getAngle() - zeroYaw;
    SmartDashboard.putNumber("navx - zeroYaw", gyroAngle);
    gyroAngle %= 360;

    if(gyroAngle < 0) 
      gyroAngle += 360;
    //Sets a deadband of .03 and adjusts the controller values so they don't jump from 0 to .3 and
    //instead are out of the remaining .97 range
    if(Math.abs(x) < .03)
      x = 0.0;
    else if(x > 0)
      x = (x - .05)/.97;
    else
      x = (x + .05)/.97;

    if(Math.abs(y) < .03)
      y = 0.0;
    else if(y > 0)
      y = (y - .05)/.97;
    else
      y = (y + .05)/.97;
    
    if(Math.abs(w) < 0.01)
      w = 0.0;
    else if(w > 0)
      w = (w - .05)/.99;
    else
      w = (w + .05)/.99;
    
    //Squares the values to slow down the speeds closer to the center
    x *= Math.abs(x);
    y *= Math.abs(y);
    w *= Math.abs(w);

    if(slowMode) {
      x *= .5;
      y *= .5;
    }
    slowMode = false;

    //w * 0.7 limits rotational speed
    driveBase.set(x, y*-1, w*.7, gyroAngle);

    if(controller.getRawButton(back) && letUpBack) {
      zeroYaw = navx.getAngle() % 360;
      if(zeroYaw < 0)
        zeroYaw += 360;
      letUpBack = false;
    } else if(!controller.getRawButton(back) && !letUpBack) {
      letUpBack = true;
    }
    //actuation code for arm
    if(controller.getRawButtonPressed(A) && letUpX)
    {
      if(armDown == false) {
        arm.set(Value.kReverse);
        armDown = true;
      } else {
        arm.set(Value.kForward);
        armDown = false;
      }
      letUpX = false;
    } else if(!controller.getRawButton(A)) {
      letUpX = true;
    }
    //actuation code for spinner
    if(controller.getRawButtonPressed(start) && letUpStart)
    {
      if(spinnerDown == false) {
        spinner.set(Value.kForward);
        spinnerDown = true;
      } else {
        spinner.set(Value.kReverse);
        spinnerDown = false;
        slowMode = true;
      }
      letUpB = false;
    } else if(!controller.getRawButton(start)) {
      letUpB = true;
    }

    //Hood
    if(controller.getRawButtonPressed(B) && letUpB)
    {
      if(hoodDown == false) {
        hood.set(Value.kReverse);
        hoodDown = true;
      } else {
        hood.set(Value.kForward);
        hoodDown = false;
      }
      letUpStart = false;
    } else if(!controller.getRawButton(B)) {
      letUpStart = true;
    }

    //85000 at low power(.1 or .2)
    if(hookLift.getSelectedSensorPosition() > -865000 && controller.getRawButton(Y)) { //Y = up
      if(hookLift.getSelectedSensorPosition() < -795000)
        hookLift.set(ControlMode.PercentOutput, -0.2);
      else
        hookLift.set(ControlMode.PercentOutput, -0.6);
    //0 at low power(.1)
    }else if(hookLift.getSelectedSensorPosition() < 2000 && controller.getRawButton(X)) {
      if(hookLift.getSelectedSensorPosition() > -25000)
        hookLift.set(ControlMode.PercentOutput, 0.1);
      else
        hookLift.set(ControlMode.PercentOutput, 0.6);
    } else
      hookLift.set(ControlMode.PercentOutput, 0);

    //slows down robot when the hook is up
    if(hookLift.getSelectedSensorPosition() < -100000) {
      slowMode = true;
    }

    //Shooter
    if(controller.getRawButton(leftBumper)) {
      shooterSpeed = 4200.0;
    } else {
      shooterPID.setIAccum(0);
    }
    
    //Shoot function
    if(controller.getRawButton(rightBumper) && shooterEncoder.getVelocity() > 2800){
      outtakeSpeed = -.6;
      carouselSpeed = -40.0;
    } 
    
    //Intake Functions
    if(controller.getRawButton(leftTrigger)) {
      intake.set(ControlMode.PercentOutput, -1);
    }
    else if(controller.getRawButton(rightTrigger)) {
      intake.set(ControlMode.PercentOutput, .8);
      carouselSpeed = -40.0;
      outtakeSpeed = .06;
    } else 
      intake.set(ControlMode.PercentOutput, 0);

    if(controller.getPOV() == 0 && Timer.getMatchTime() <= 30) {
      winch1.set(ControlMode.PercentOutput, 1);
      winch2.set(ControlMode.Follower, 14);
    } else {
      winch1.set(ControlMode.PercentOutput, 0);
      winch2.set(ControlMode.Follower, 14);
    }
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());

    /*Function to unjam a ball on the Carousel.
      If the carousel is within 3 counts of it's last position for 1/4 of a sec when it should be moving this moves
      the carousol backwards for 1/2 a second*/
    if(unjam) {
      if(tim.get() < .5)
        carouselSpeed = 40.0;
      else
        unjam = false;
    } else if(carouselSpeed != 0 && Math.abs(carousel.getSelectedSensorPosition()-prevCarouselPos) < 3) {
      if(!timStart) {
        tim.start();
        timStart = true;
      }
      if(tim.get() > .25) {
        unjam = true;
        tim.reset();
      }
    } else if(timStart && !unjam) {
      tim.stop();
      tim.reset();
      timStart = false;
    }
    prevCarouselPos = carousel.getSelectedSensorPosition();

    if(carouselSpeed == 0){
      carousel.set(ControlMode.PercentOutput, 0);
    } else /*if(carouselVelPID)*/ {
      carousel.config_kP(0, carokP);
      carousel.config_kI(0, carokI);
      carousel.config_kD(0, carokD);
      carousel.set(ControlMode.Velocity, carouselSpeed);
    }

    outtake.set(ControlMode.PercentOutput, outtakeSpeed);
    
    if(shooterSpeed == 0)
      shooter.set(0);
    else
      shooterPID.setReference(shooterSpeed, ControlType.kVelocity);

    //SmartDashboard commands
    /*SmartDashboard.putNumber("CarouselP", caroPoskP);
    SmartDashboard.putNumber("CarouselI", caroPoskI);
    SmartDashboard.putNumber("CarouselD", caroPoskD);
    SmartDashboard.putNumber("CarouselSpeed", carouselSpeed);
    SmartDashboard.putNumber("Carousel I Accum", carousel.getIntegralAccumulator());
    SmartDashboard.putBoolean("carouselVelPID", carouselVelPID);
    SmartDashboard.putNumber("velocity", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("carousel Pos", carousel.getSelectedSensorPosition());
    SmartDashboard.putNumber("carousel Vel", carousel.getSelectedSensorVelocity());
    SmartDashboard.putNumber("HookLift", hookLift.getSelectedSensorPosition());
    */driveBase.displaySmartDashboard(true, true, true, true);
    /*SmartDashboard.putNumber("Zero Yaw", zeroYaw);
    SmartDashboard.putNumber("Gyro", navx.getAngle());
    SmartDashboard.putNumber("X", x);
    SmartDashboard.putNumber("Y", y);
    SmartDashboard.putNumber("W", w);*/
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    /*
    //845000 at low power(.1 or .2)
    if(hookLift.getSelectedSensorPosition() > -825000 && controller.getRawButton(Y)) { //Y = up
      hookLift.set(ControlMode.PercentOutput, -0.5);
    //0 at low power(.1)
    }else if(hookLift.getSelectedSensorPosition() < -25000 && controller.getRawButton(A)) { //A = down
      hookLift.set(ControlMode.PercentOutput, 0.5);
    } else
      hookLift.set(ControlMode.PercentOutput, 0);

    SmartDashboard.putNumber("HookLift", hookLift.getSelectedSensorPosition());*/

    if(controller.getRawButton(X)) {
      if(!timStart) {
        tim.start();
        timStart = true;
      }
      shooterPID.setReference(3500, ControlType.kVelocity);
      if(shooterEncoder.getVelocity() > 3400 && shooterEncoder.getVelocity() < 3600) {
        System.out.println(tim.get());
        tim.stop();
      }
    } else 
      shooter.set(0);

    SmartDashboard.putNumber("velocity", shooterEncoder.getVelocity());
    
    if(Math.abs(controller.getRawAxis(1)) > .05){
      winch1.set(ControlMode.PercentOutput, controller.getRawAxis(1));
      winch2.set(ControlMode.Follower, 14);
    } else {
      winch1.set(ControlMode.PercentOutput, 0);
      winch2.set(ControlMode.Follower, 14);
    }
    
    /*SmartDashboard.putNumber("Red",colorSensor.getRed());
    SmartDashboard.putNumber("Blue", colorSensor.getBlue());
    SmartDashboard.putNumber("Green", colorSensor.getGreen());
    SmartDashboard.putNumber("Proximity", colorSensor.getProximity());*/
  }
}