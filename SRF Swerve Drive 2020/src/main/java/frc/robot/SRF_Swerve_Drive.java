package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class SRF_Swerve_Drive {
    
    private double wheelBase, trackWidth, radius;

    private double[] wheelSpeed = new double[4];
    private double[] wheelAngle = new double[4];

    private SRF_Swerve_Module frontLeftModule;
    private SRF_Swerve_Module frontRightModule;
    private SRF_Swerve_Module rearLeftModule;
    private SRF_Swerve_Module rearRightModule;

    public SRF_Swerve_Drive(SRF_Swerve_Module frontLeft, SRF_Swerve_Module frontRight, SRF_Swerve_Module rearLeft,
                            SRF_Swerve_Module rearRight, double w, double t, double r) {
        frontLeftModule = frontLeft;
        frontRightModule = frontRight;
        rearLeftModule = rearLeft;
        rearRightModule = rearRight;
        wheelBase = w;
        trackWidth = t;
        radius = r;
    }

    /*
    Changes the X,Y, values inputed (Like from a controller joystick) so they are oriented 
    to forward being the opposite end of the field from you as opposed to the front of the robot
    */
	private double[] convertToFieldPosition(double X, double Y, double gyroAngle){
        double newY, newX, temp;
        
        gyroAngle = (gyroAngle/180) * Math.PI;
        //SmartDashboard.putNumber("angle at Convert", gyroAngle);

        temp = Y*Math.cos(gyroAngle) + X*Math.sin(gyroAngle);
        newX = Y*-1*Math.sin(gyroAngle) + X*Math.cos(gyroAngle);
        newY = temp;
        //SmartDashboard.putNumber("newX", newX);
        //SmartDashboard.putNumber("newY", newY);

        return new double[] {newX,newY};
    }

    //Sets and runs the calculate function (see below) with the appropriate X, Y, and Rotation
    public void set(double X, double Y, double W) {
        calculate(X,Y,W);
    }

    //Does the same as above except changing the values to be field oriented
    public void set(double X, double Y, double W, double gyroAngle) {
        double[] newCoordinates = convertToFieldPosition(X,Y,gyroAngle);
        X = newCoordinates[0];
        Y = newCoordinates[1];
        calculate(X,Y,W);
    }
    
    private void calculate(double X, double Y, double W){
        double A, B, C, D;
        double greatestValue = -1;
        
        A = X - W*(wheelBase/radius);
        B = X + W*(wheelBase/radius);
        C = Y - W*(trackWidth/radius);
        D = Y + W*(trackWidth/radius);

        wheelSpeed[0] = (Math.sqrt(Math.pow(B,2) + Math.pow(C,2)));
        wheelSpeed[1] = Math.sqrt(Math.pow(B,2) + Math.pow(D,2));
        wheelSpeed[2] = Math.sqrt(Math.pow(A,2) + Math.pow(D,2));
        wheelSpeed[3] = Math.sqrt(Math.pow(A,2) + Math.pow(C,2));
            
        //if no controller input keep current angle(Math inside would change) but change speed to zero(Math above)
        if(X != 0 || Y != 0 || W != 0) {
            /*
            Finds the largest wheel speed greater then 1 and converts all speeds so they are proportional
            to the largest being 1. It does this since the range of the motor is [-1,1].
            Note: the values from the equation above for the wheel speed are only positive
            */
            for(int wheel = 0; wheel < 4; wheel++) {
                if(wheelSpeed[0] > 1 && wheelSpeed[0] > greatestValue) {
                    greatestValue = wheelSpeed[0];
                }
            }
            if(greatestValue > -1) {
                for(int wheel = 0; wheel < 4; wheel++) {
                    wheelSpeed[0] = wheelSpeed[0]/greatestValue;
                }
            }

            //gives angle of the wheels in degrees
            wheelAngle[0] = Math.atan2(B,C)*(180/Math.PI);
            wheelAngle[1] = Math.atan2(B,D)*(180/Math.PI);
            wheelAngle[2] = Math.atan2(A,D)*(180/Math.PI);
            wheelAngle[3] = Math.atan2(A,C)*(180/Math.PI);
        }
        frontRightModule.set(wheelAngle[0], wheelSpeed[0]);
        frontLeftModule.set(wheelAngle[1], wheelSpeed[1]);
        rearLeftModule.set(wheelAngle[2], wheelSpeed[2]);
        rearRightModule.set(wheelAngle[3], wheelSpeed[3]);
    }

    public void setSpeedZero(){
        wheelSpeed[0] = 0;
        wheelSpeed[1] = 0;
        wheelSpeed[2] = 0;
        wheelSpeed[3] = 0;
    }

    public void displaySmartDashboard(boolean showAngle, boolean showSpeed, boolean showRotEncoder, 
                                      boolean showPIDTarget){
        if(showAngle) {
            //Angle in degrees
            SmartDashboard.putNumber("Front Left Angle", getFrontLeftAngle());
            SmartDashboard.putNumber("Front Right Angle", getFrontRightAngle());
            SmartDashboard.putNumber("Rear Left Angle", getRearLeftAngle());
            SmartDashboard.putNumber("Rear Right Angle",getRearRightAngle());
        }
        if(showSpeed) {
            SmartDashboard.putNumber("Front Left Speed", getFrontLeftSpeed());
            SmartDashboard.putNumber("Front Right Speed", getFrontRightSpeed());
            SmartDashboard.putNumber("Rear Left Speed", getRearLeftSpeed());
            SmartDashboard.putNumber("Rear Right Speed", getRearRightSpeed());
        }
        if(showRotEncoder) {
            SmartDashboard.putNumber("Front Left Sensor", frontLeftModule.getSensorValue());
            SmartDashboard.putNumber("Front Right Sensor", frontRightModule.getSensorValue());
            SmartDashboard.putNumber("Rear Left Sensor", rearLeftModule.getSensorValue());
            SmartDashboard.putNumber("Rear Right Sensor", rearRightModule.getSensorValue());
        }
        if(showPIDTarget) {
            SmartDashboard.putNumber("Front Left PID Target", frontLeftModule.getPIDTarget());
            SmartDashboard.putNumber("Front Right PID Target", frontRightModule.getPIDTarget());
            SmartDashboard.putNumber("Rear Left PID Target", rearLeftModule.getPIDTarget());
            SmartDashboard.putNumber("Rear Right PID Target", rearRightModule.getPIDTarget());
        }
    }

    public double[] getWheelAngles(){
        return new double[] {wheelAngle[0], wheelAngle[1], wheelAngle[2], wheelAngle[3]};
    }

    public double getFrontLeftSpeed(){
        return wheelSpeed[1];
    }

    public double getFrontRightSpeed(){
        return wheelSpeed[0];
    }

    public double getRearLeftSpeed(){
        return wheelSpeed[2];
    }

    public double getRearRightSpeed(){
        return wheelSpeed[3];
    }

    public double getFrontLeftAngle(){
        return wheelAngle[1];
    }

    public double getFrontRightAngle(){
        return wheelAngle[0];
    }

    public double getRearLeftAngle(){
        return wheelAngle[2];
    }

    public double getRearRightAngle(){
        return wheelAngle[3];
    }
}