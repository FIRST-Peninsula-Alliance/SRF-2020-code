/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
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
  TalonSRX rotationController;
  CANSparkMax wheel;
  SRF_Swerve_Drive driveBase;
  AHRS navx;

  double kP = 12, kI = 0.065, kD = 0;

  boolean letUpOne;

  boolean testRotationController = true;

  @Override
  public void robotInit() {
    navx = new AHRS();
    wheel = new CANSparkMax(2, MotorType.kBrushless);
    rotationController = new TalonSRX(1);
    driveBase = new SRF_Swerve_Drive(30.0, 24.0, 38.42);   
    
    rotationController.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    rotationController.config_kP(0, kP);
    rotationController.config_kI(0, kI);
    rotationController.config_kD(0, kD);
    rotationController.setSelectedSensorPosition(0);
  }   
 
  @Override
  public void autonomousInit() { 
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    letUpOne = true;
    //rotationController.setSelectedSensorPosition(0);
  }


  @Override
  public void teleopPeriodic() {
    driveBase.set(controller.getRawAxis(0), controller.getRawAxis(1), controller.getRawAxis(2), navx.getAngle());

    if(Math.abs(driveBase.getTopRightSpeed()) > .05 && !testRotationController)
      wheel.set(driveBase.getTopRightSpeed());
    
    //about 512 counts/half rotation of wheel Gear
    if(Math.abs(controller.getRawAxis(0)) > .05 || Math.abs(controller.getRawAxis(1)) > .05 || Math.abs(controller.getRawAxis(2)) > .05)
      rotationController.set(ControlMode.Position, Math.round((driveBase.getTopRightAngle()/360)*1040));

    if(controller.getRawButton(1) && letUpOne) {
      navx.setAngleAdjustment(90);
      letUpOne = false;
    } else if(!controller.getRawButton(1)) {
      navx.setAngleAdjustment(0);
      letUpOne = true;
    }

    SmartDashboard.putNumber("Tire Speed", driveBase.getTopRightSpeed());
    SmartDashboard.putNumber("Angle in Degrees",driveBase.getTopRightAngle());
    SmartDashboard.putNumber("PID Target", Math.round((driveBase.getTopRightAngle() / 360) * 1040));
    SmartDashboard.putNumber("Rotation Encoder Value", rotationController.getSelectedSensorPosition());
    SmartDashboard.putNumber("Gyro", navx.getAngle());
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {

  }

}
