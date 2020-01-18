package frc.robot;

import frc.robot.SRF_Swerve_Drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;

class test {
    static AHRS n;
    public static void main(String arg[]) {
        SRF_Swerve_Drive swerve = new SRF_Swerve_Drive(30, 24, 38.41875, n);
        double[][] wheelArray = swerve.calculate(false, .34, 1, .25);
        for(double[] wheel: wheelArray) {
            for(double i: wheel){
                System.out.println(i);
            }
        }
    }
}