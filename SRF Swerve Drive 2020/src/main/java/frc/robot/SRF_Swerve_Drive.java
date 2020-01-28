package frc.robot;

class SRF_Swerve_Drive {
    
    double wheelBase, trackWidth, radius, gyroValue;

    double[] wheelSpeed = new double[4];
    double[] wheelAngle = new double[4];

    public SRF_Swerve_Drive(double w, double t, double r) {
        wheelBase = w;
        trackWidth = t;
        radius = r;
    }

    /*
    Changes the X,Y, values inputed (Like from a controller joystick) so they are oriented 
    to forward being the opposite end of the field from you as opposed to the front of the robot
    */
	private double[] convertToFieldPosition(double Y, double X, double gyroAngle){
        double newY, newX, temp;

        temp = Y*Math.cos(gyroAngle) + X*Math.sin(gyroAngle);
        newX = Y*-1*Math.sin(gyroAngle) + X*Math.cos(gyroAngle);
        newY = temp;

        return new double[] {newX,newY};
    }

    //Sets and runs the calculate function (see below) with the appropriate X, Y, and Rotation
    public void set(double X, double Y, double W) {
        calculate(X,Y,W);
    }

    //Does the same as above except changing the values to be field oriented
    public void set(double X, double Y, double W, double gyroAngle) {
        double[] newCoordinates = convertToFieldPosition(X,Y,(gyroAngle/180)*Math.PI);
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

        wheelSpeed[0] = Math.sqrt(Math.pow(B,2) + Math.pow(C,2));
        wheelSpeed[1] = Math.sqrt(Math.pow(B,2) + Math.pow(D,2));
        wheelSpeed[2] = Math.sqrt(Math.pow(A,2) + Math.pow(D,2));
        wheelSpeed[3] = Math.sqrt(Math.pow(A,2) + Math.pow(C,2));

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









