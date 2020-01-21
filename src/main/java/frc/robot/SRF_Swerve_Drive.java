package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;

class SRF_Swerve_Drive {
    Joystick xBox;
    AHRS navx;
    double wheelBase,trackWidth,radius, gyroValue;
    /*
    wheelbase[]: Wheel number
    wheelbase[][]: 0 - Speed, 1 - Angle
    */
    double[][] wheelValues = new double[4][2];
    
    public SRF_Swerve_Drive(Joystick j, double w, double t, double r, AHRS gyro) {
        xBox = j;
        wheelBase = w;
        trackWidth = t;
        radius = r;
        navx = gyro;
    }

    public SRF_Swerve_Drive(double w, double t, double r, AHRS gyro) {
        wheelBase = w;
        trackWidth = t;
        radius = r;
        navx = gyro;
    }

    public double[] convertToFieldPosition(double Y, double X, double gyroAngle){
        double newY, newX, temp;

        temp = Y*Math.cos(gyroAngle) + X*Math.sin(gyroAngle);
        newX = Y*-1*Math.sin(gyroAngle) + X*Math.cos(gyroAngle);
        newY = temp;

        return new double[] {newX,newY};
    }

    public double[][] calculate(boolean orientToField){
        double X, Y, W, A, B, C, D;
        double greatestValue = -1;
        X = xBox.getRawAxis(0);
        Y = xBox.getRawAxis(1);
        W = xBox.getRawAxis(2);
        
        if(orientToField) {
            double[] newCoordinates = convertToFieldPosition(X,Y,(navx.getAngle()/180)*Math.PI);
            X = newCoordinates[0];
            Y = newCoordinates[1];
        }

        A = X - W*(wheelBase/radius);
        B = X + W*(wheelBase/radius);
        C = Y - W*(trackWidth/radius);
        D = Y + W*(trackWidth/radius);

        wheelValues[0][0] = Math.sqrt(Math.pow(B,2) + Math.pow(C,2));
        wheelValues[1][0] = Math.sqrt(Math.pow(B,2) + Math.pow(D,2));
        wheelValues[2][0] = Math.sqrt(Math.pow(A,2) + Math.pow(D,2));
        wheelValues[3][0] = Math.sqrt(Math.pow(A,2) + Math.pow(C,2));

        for(int wheel = 0; wheel < 4; wheel++) {
            if(wheelValues[wheel][0] > 1 && wheelValues[wheel][0] > greatestValue) {
                greatestValue = wheelValues[wheel][0];
            }
        }

        if(greatestValue > -1) {
            for(int wheel = 0; wheel < 4; wheel++) {
                wheelValues[wheel][0] = wheelValues[wheel][0]/greatestValue;
            }
        }

        wheelValues[0][1] = Math.atan2(B,C)*(180/Math.PI);
        wheelValues[1][1] = Math.atan2(B,D)*(180/Math.PI);
        wheelValues[2][1] = Math.atan2(A,D)*(180/Math.PI);
        wheelValues[3][1] = Math.atan2(A,C)*(180/Math.PI);

        return wheelValues;
    }

    public double[][] calculate(boolean orientToField, double X, double Y, double W){
        double A, B, C, D;
        double greatestValue = -1;
        
        if(orientToField) {
            double[] newCoordinates = convertToFieldPosition(X,Y,(navx.getAngle()/180)*Math.PI);
            X = newCoordinates[0];
            Y = newCoordinates[1];
        }

        A = X - W*(wheelBase/radius);
        B = X + W*(wheelBase/radius);
        C = Y - W*(trackWidth/radius);
        D = Y + W*(trackWidth/radius);

        wheelValues[0][0] = Math.sqrt(Math.pow(B,2) + Math.pow(C,2));
        wheelValues[1][0] = Math.sqrt(Math.pow(B,2) + Math.pow(D,2));
        wheelValues[2][0] = Math.sqrt(Math.pow(A,2) + Math.pow(D,2));
        wheelValues[3][0] = Math.sqrt(Math.pow(A,2) + Math.pow(C,2));

        for(int wheel = 0; wheel < 4; wheel++) {
            if(wheelValues[wheel][0] > 1 && wheelValues[wheel][0] > greatestValue) {
                greatestValue = wheelValues[wheel][0];
            }
        }

        if(greatestValue > -1) {
            for(int wheel = 0; wheel < 4; wheel++) {
                wheelValues[wheel][0] = wheelValues[wheel][0]/greatestValue;
            }
        }

        wheelValues[0][1] = Math.atan2(B,C)*(180/Math.PI);
        wheelValues[1][1] = Math.atan2(B,D)*(180/Math.PI);
        wheelValues[2][1] = Math.atan2(A,D)*(180/Math.PI);
        wheelValues[3][1] = Math.atan2(A,C)*(180/Math.PI);

        return wheelValues;
    }
}









