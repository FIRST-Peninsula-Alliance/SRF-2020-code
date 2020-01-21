package frc.robot;

class SRF_Swerve_Drive {
    
    double wheelBase, trackWidth, radius, gyroValue;
    /*T:Top B:Bottom
      L:Left R:Right
      S: Speed R: Rotation*/
    double[] wheelSpeed = new double[4];
    double[] wheelAngle = new double[4];

    /*wheelbase[]: Wheel number
      wheelbase[][]: 0 - Speed, 1 - Angle*/

    public SRF_Swerve_Drive(double w, double t, double r) {
        wheelBase = w;
        trackWidth = t;
        radius = r;
    }

	private double[] convertToFieldPosition(double Y, double X, double gyroAngle){
        double newY, newX, temp;

        temp = Y*Math.cos(gyroAngle) + X*Math.sin(gyroAngle);
        newX = Y*-1*Math.sin(gyroAngle) + X*Math.cos(gyroAngle);
        newY = temp;

        return new double[] {newX,newY};
    }

    public void set(double X, double Y, double W) {
        calculate(X,Y,W);
    }

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

        wheelAngle[0] = Math.atan2(B,C)*(180/Math.PI);
        wheelAngle[1] = Math.atan2(B,D)*(180/Math.PI);
        wheelAngle[2] = Math.atan2(A,D)*(180/Math.PI);
        wheelAngle[3] = Math.atan2(A,C)*(180/Math.PI);
    }

    public double getTopLeftSpeed(){
        return wheelSpeed[1];
    }

    public double getTopRightSpeed(){
        return wheelSpeed[0];
    }

    public double getBottomLeftSpeed(){
        return wheelSpeed[2];
    }

    public double getBottomRightSpeed(){
        return wheelSpeed[3];
    }

    public double getTopLeftAngle(){
        return wheelAngle[1];
    }

    public double getTopRightAngle(){
        return wheelAngle[0];
    }

    public double getBottomLeftAngle(){
        return wheelAngle[2];
    }

    public double getBottomRightAngle(){
        return wheelAngle[3];
    }
}









