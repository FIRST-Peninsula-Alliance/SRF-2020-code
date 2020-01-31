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

  SRF_Swerve_Drive driveBase;

  AHRS navx;
  Block pixyBlock;
  PixyCam pixy = new PixyCam();

  double kP = 12, kI = 0.065, kD = 0;

  boolean testRotationController = true;

  Boolean letUpB = true;

  I2C wire = new I2C(Port.kOnboard, 0x0);

  @Override
  public void robotInit() {
    navx = new AHRS();

    //Replace -1 with actual values
    frontLeft = new CANSparkMax(2, MotorType.kBrushless);
    frontRight = new CANSparkMax(8, MotorType.kBrushless);
    rearLeft = new CANSparkMax(4, MotorType.kBrushless);
    rearRight = new CANSparkMax(6, MotorType.kBrushless);

    frontLeftRot = new TalonSRX(1);
    frontRightRot = new TalonSRX(7);
    rearLeftRot = new TalonSRX(3);
    rearRightRot = new TalonSRX(5);

    driveBase = new SRF_Swerve_Drive(25.0, 21.0, 32.65);

    frontLeftRot.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    frontLeftRot.config_kP(0, kP);
    frontLeftRot.config_kI(0, kI);
    frontLeftRot.config_kD(0, kD);
    frontLeftRot.setSelectedSensorPosition(0);

    frontRightRot.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    frontRightRot.config_kP(0, kP);
    frontRightRot.config_kI(0, kI);
    frontRightRot.config_kD(0, kD);
    frontRightRot.setSelectedSensorPosition(0);

    rearLeftRot.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    rearLeftRot.config_kP(0, kP);
    rearLeftRot.config_kI(0, kI);
    rearLeftRot.config_kD(0, kD);
    rearLeftRot.setSelectedSensorPosition(0);

    rearRightRot.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    rearRightRot.config_kP(0, kP);
    rearRightRot.config_kI(0, kI);
    rearRightRot.config_kD(0, kD);
    rearRightRot.setSelectedSensorPosition(0);
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
    
    driveBase.set(controller.getRawAxis(0), controller.getRawAxis(1), controller.getRawAxis(2), navx.getAngle());

    // about 512 counts/half rotation of wheel Gear
    if (Math.abs(controller.getRawAxis(0)) > .05 || Math.abs(controller.getRawAxis(1)) > .05  || Math.abs(controller.getRawAxis(2)) > .05) {
      /*frontLeft.set(driveBase.getFrontLeftSpeed());
      frontRight.set(driveBase.getFrontRightSpeed());
      rearLeft.set(driveBase.getRearLeftSpeed());
      rearRight.set(driveBase.getRearRightSpeed());*/

      frontRightRot.set(ControlMode.Position, Math.round((driveBase.getFrontLeftAngle() / 360) * 1040));
      frontLeftRot.set(ControlMode.Position, Math.round((driveBase.getFrontRightAngle() / 360) * 1040));
      rearLeftRot.set(ControlMode.Position, Math.round((driveBase.getRearLeftAngle() / 360) * 1040));
      rearRightRot.set(ControlMode.Position, Math.round((driveBase.getRearRightAngle() / 360) * 1040));
    } else {
      /*frontLeft.set(0);
      frontRight.set(0);
      rearLeft.set(0);
      rearRight.set(0);*/
    }

    if(controller.getRawButton(1) && letUpB) {
      navx.
      letUpB = false;
    }
    if(!controller.getRawButton(1) && !letUpB) {
      letUpB = true;
    }


    SmartDashboard.putNumber("Front Left Speed", driveBase.getFrontLeftSpeed());
    SmartDashboard.putNumber("Front Right Speed", driveBase.getFrontRightSpeed());
    SmartDashboard.putNumber("Rear Left Speed", driveBase.getRearLeftSpeed());
    SmartDashboard.putNumber("Rear Right Speed", driveBase.getRearRightSpeed());

    //Angle of the wheels in Degrees
    SmartDashboard.putNumber("Front Left Angle", driveBase.getFrontLeftAngle());
    SmartDashboard.putNumber("Front Right Angle", driveBase.getFrontRightAngle());
    SmartDashboard.putNumber("Rear Left Angle", driveBase.getRearLeftAngle());
    SmartDashboard.putNumber("Rear Right Angle", driveBase.getRearRightAngle());

    SmartDashboard.putNumber("Front Left PID Target", Math.round((driveBase.getFrontLeftAngle() / 360) * 1040));
    SmartDashboard.putNumber("Front Right PID Target", Math.round((driveBase.getFrontRightAngle() / 360) * 1040));
    SmartDashboard.putNumber("Rear Left PID Target", Math.round((driveBase.getRearLeftAngle() / 360) * 1040));
    SmartDashboard.putNumber("Rear Right PID Target", Math.round((driveBase.getRearRightAngle() / 360) * 1040));

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