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

  @Override
  public void robotInit() {
    navx = new AHRS();
    wheel = new CANSparkMax(2, MotorType.kBrushless);
    rotationController = new TalonSRX(1);
    driveBase = new SRF_Swerve_Drive(controller, 30.0, 24.0, 38.42, navx);   
    
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
    double[][] values = driveBase.calculate(true);

    /*

    if(Math.abs(controller.getRawAxis(3)) > .1)
      wheel.set(controller.getRawAxis(3));
    else
      wheel.set(0);
    */
    
    /*System.out.println(values[1][1]);
    System.out.println(values[2][1]);
    System.out.println(values[3][1]);*/
    
    if(controller.getRawButton(1) && letUpOne) {
      navx.setAngleAdjustment(90);
      letUpOne = false;
    } else if(!controller.getRawButton(1)) {
      navx.setAngleAdjustment(0);
      letUpOne = true;
    }

    
    
    //wheel.set(values[0][0]);

    //about 512 counts/half rotation of wheel Gear
    if(Math.abs(controller.getRawAxis(0)) > .2 || Math.abs(controller.getRawAxis(1)) > .2 || Math.abs(controller.getRawAxis(2)) > .2)
      rotationController.set(ControlMode.Position, Math.round((values[0][1]/360)*1040));
    else
      rotationController.set(ControlMode.Position, 0);
    
    SmartDashboard.putNumber("Tire Speed",values[0][0]);
    SmartDashboard.putNumber("Angle in Degrees",values[0][1]);
    SmartDashboard.putNumber("PID Target", Math.round((values[0][1] / 360) * 1040));
    SmartDashboard.putNumber("Sensor Value", rotationController.getSelectedSensorPosition());
    SmartDashboard.putNumber("Gyro", navx.getAngle());
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {

  }

}
