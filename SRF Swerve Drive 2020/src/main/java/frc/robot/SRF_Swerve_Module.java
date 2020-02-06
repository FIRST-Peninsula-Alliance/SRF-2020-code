package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRF_Swerve_Module {    
    TalonSRX rotationMotor;
    CANSparkMax speedMotor;

    final int countsPerRev = 1024;

    private double PIDTarget;

    public SRF_Swerve_Module(int talonID, int sparkID, double P, double I, double D) {
        rotationMotor = new TalonSRX(talonID);
        speedMotor = new CANSparkMax(sparkID, MotorType.kBrushless);

        rotationMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        rotationMotor.config_kP(0, P);
        rotationMotor.config_kI(0, I);
        rotationMotor.config_kD(0, D);
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
        
        
        speedMotor.set(speed);

        SmartDashboard.putNumber("Distance Between", distanceBetween);
        PIDTarget = rotationMotor.getSelectedSensorPosition() - distanceBetween;
    }

    public double getPIDTarget() {
        return PIDTarget;
    }
    
    public int getSensorValue() {
        return rotationMotor.getSelectedSensorPosition();
    }
}