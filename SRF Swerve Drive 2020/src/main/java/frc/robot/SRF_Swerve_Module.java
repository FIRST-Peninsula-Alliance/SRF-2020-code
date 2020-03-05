package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRF_Swerve_Module {    
    TalonSRX rotationMotor;
    CANSparkMax speedMotor;

    CANPIDController speedPID;

    double speedP = 5e-5, speedI = 1e-6, speedD = 0;
    int zeroOffset;
    final int countsPerRev = 1024;

    private double PIDTarget;

    public SRF_Swerve_Module(int talonID, int sparkID, double P, double I, double D, int offset) {
        rotationMotor = new TalonSRX(talonID);
        
        rotationMotor.setNeutralMode(NeutralMode.Coast);
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        rotationMotor.config_kP(0, P);
        rotationMotor.config_kI(0, I);
        rotationMotor.config_kD(0, D);
        
        speedMotor = new CANSparkMax(sparkID, MotorType.kBrushless);
        speedMotor.setIdleMode(IdleMode.kBrake);
        speedMotor.setSmartCurrentLimit(50);
        speedMotor.setOpenLoopRampRate(.35);
        speedPID = new CANPIDController(speedMotor);
        speedPID.setP(speedP);
        speedPID.setI(speedI);
        speedPID.setD(speedD);

        zeroOffset = offset;
    }

    public void set(double angle, double speed) {
        int currentAngle = rotationMotor.getSelectedSensorPosition();
        double distanceBetween;
        int sign = 1;

        currentAngle %= countsPerRev;
        if(angle < 0)
            currentAngle += countsPerRev;
        //SmartDashboard.putNumber("% counts/Rev", currentAngle);

        angle = (angle/360)*1024;
        angle += zeroOffset;
        if(angle < 0)
            angle += countsPerRev;
        //SmartDashboard.putNumber("Angle in Rev", angle);
        
        distanceBetween = angle - currentAngle;
        if(distanceBetween < 0)
            distanceBetween += countsPerRev;
        //SmartDashboard.putNumber("Init DistBetween", distanceBetween);
        
        if(distanceBetween > (countsPerRev - distanceBetween)) {
            distanceBetween = countsPerRev - distanceBetween;
            sign *= -1;
        }


        if(distanceBetween > 256) {
            distanceBetween = 512 - distanceBetween;
            sign *= -1;
            speed *= -1;
            //SmartDashboard.putNumber("distBetween Changed", distanceBetween);
        }

        if(Math.abs(distanceBetween) > 10)
            rotationMotor.set(ControlMode.Position, rotationMotor.getSelectedSensorPosition() + distanceBetween * sign);
        
        speedPID.setReference(speed, ControlType.kDutyCycle);

        //SmartDashboard.putNumber("Distance Between", distanceBetween);
        PIDTarget = rotationMotor.getSelectedSensorPosition() - distanceBetween;
    }

    public double getPIDTarget() {
        return PIDTarget;
    }
    
    public int getSensorValue() {
        return rotationMotor.getSelectedSensorPosition();
    }
}