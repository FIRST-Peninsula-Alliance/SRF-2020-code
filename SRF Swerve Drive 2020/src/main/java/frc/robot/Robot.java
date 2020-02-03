/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

  Joystick controller = new Joystick(0);

  TalonSRX frontLeftRot, frontRightRot, rearLeftRot, rearRightRot;
  CANSparkMax frontLeft, frontRight, rearLeft, rearRight;

  SRF_Swerve_Module FLModule;
  SRF_Swerve_Module FRModule;
  SRF_Swerve_Module RLModule;
  SRF_Swerve_Module RRModule;
  SRF_Swerve_Drive driveBase;

  AHRS navx;
  Block pixyBlock;
  PixyCam pixy = new PixyCam();

  double kP = 16, kI = 0.00013, kD = 0;

  //Joystick values for swerve drive
  double x, y, w;

  boolean testRotationController = true;

  Boolean letUpB = true;

  I2C wire = new I2C(Port.kOnboard, 0x0);

  @Override
  public void robotInit() {
    navx = new AHRS();

    frontLeft = new CANSparkMax(2, MotorType.kBrushless);
    frontRight = new CANSparkMax(8, MotorType.kBrushless);
    rearLeft = new CANSparkMax(4, MotorType.kBrushless);
    rearRight = new CANSparkMax(6, MotorType.kBrushless);

    frontLeftRot = new TalonSRX(1);
    frontRightRot = new TalonSRX(7);
    rearLeftRot = new TalonSRX(3);
    rearRightRot = new TalonSRX(5);

    FLModule = new SRF_Swerve_Module(1, 2, kP, kI, kD);
    FRModule = new SRF_Swerve_Module(7, 8, kP, kI, kD);
    RLModule = new SRF_Swerve_Module(3, 4, kP, kI, kD);
    RRModule = new SRF_Swerve_Module(5, 6, kP, kI, kD);

    driveBase = new SRF_Swerve_Drive(FLModule, FRModule, RLModule, RRModule, 25.0, 21.0, 32.65);
  }
 
  public void disabledPeriodic(){
    /*SmartDashboard.putNumber("Front Left Sensor", frontLeftRot.getSelectedSensorPosition());
    SmartDashboard.putNumber("Front Right Sensor", frontRightRot.getSelectedSensorPosition());
    SmartDashboard.putNumber("Rear Left Sensor", rearLeftRot.getSelectedSensorPosition());
    SmartDashboard.putNumber("Rear Right Sensor", rearRightRot.getSelectedSensorPosition());*/
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    x = controller.getRawAxis(0);
    y = controller.getRawAxis(1);
    w = controller.getRawAxis(2);

    if(Math.abs(x) < .03)
      x = 0.0;
    if(Math.abs(y) < .03)
      y = 0.0;
    if(Math.abs(w) < .03)
      w = 0.0;

    //if(x != 0 || y != 0 || w != 0) {
      driveBase.set(x, y*-1, w);
      /*frontLeft.set(driveBase.getFrontLeftSpeed()/2);
      frontRight.set(driveBase.getFrontRightSpeed()/2);
      rearLeft.set(driveBase.getRearLeftSpeed()/2);
      rearRight.set(driveBase.getRearRightSpeed()/2);
    } else {
      frontLeft.set(0);
      frontRight.set(0);
      rearLeft.set(0);
      rearRight.set(0);
    }*/

    //1024 is the number of encoder counts per revolution of the wheel
    /*frontRightRot.set(ControlMode.Position, Math.round((driveBase.getFrontRightAngle() / 360) * 1024));
    frontLeftRot.set(ControlMode.Position, Math.round((driveBase.getFrontLeftAngle() / 360) * 1024));
    rearLeftRot.set(ControlMode.Position, Math.round((driveBase.getRearLeftAngle() / 360) * 1024));
    rearRightRot.set(ControlMode.Position, Math.round((driveBase.getRearRightAngle() / 360) * 1024));*/

    if(controller.getRawButton(1) && letUpB) {
      letUpB = false;
    }
    if(!controller.getRawButton(1) && !letUpB) {
      letUpB = true;
    }

    driveBase.displaySmartDashboard(true, true, true, true);
    SmartDashboard.putNumber("Gyro", navx.getAngle());
  }

  @Override
  public void testInit() {
    pixyBlock = pixy.getBlock();
  }

  @Override
  public void testPeriodic() {
    pixyBlock = pixy.getBlock();
    pixy.displaySmartDashboard();
  }
}