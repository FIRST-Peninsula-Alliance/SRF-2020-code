package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

        currentAngle = currentAngle % countsPerRev;
        if(currentAngle < 0)
            currentAngle = (countsPerRev + currentAngle);

        if(angle < 0)
            angle = (180 - angle*-1) + 180;
        angle = (angle/360) * countsPerRev;

        distanceBetween = currentAngle - angle;

        if(Math.abs(distanceBetween) > 512) {
            if(distanceBetween > 0)
                distanceBetween -= countsPerRev;
            else
                distanceBetween += countsPerRev;
            speed *= -1;
        }

        rotationMotor.set(ControlMode.Position, rotationMotor.getSelectedSensorPosition() + distanceBetween);
        speedMotor.set(speed);

        PIDTarget = rotationMotor.getSelectedSensorPosition() + distanceBetween;
    }

    public double getPIDTarget() {
        return PIDTarget;
    }
    
    public int getSensorValue() {
        return rotationMotor.getSelectedSensorPosition();
    }
}