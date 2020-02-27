package frc.robot;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PixyCam {

    //Block is an Object to hold the data from the pixy
    //Data: Signature(what block sent), X and Y coordinates of center of detected object, width, Height
    Block[] blocks = new Block[3];
    //The index of the best target
    Block currentBlock;

    I2C wire;
    //Tells if the the we've recieved a valid block from the pixy camera
    Boolean pixyHasTarget;

    //PID to have the robot follow the PixyCam target
    SRF_PID targetPID;
    double P = .01;
    double I = 0;
    double D = 0;

    /*adjust represents 1/(160 pixels/32.5 degrees) which, when a number of pixels is multiplied by this, converts to degrees
      320 is x screen resolution, 75 is camera angle of view
      each of these values are halved as the screen resolution is converted to be + or - 0 with 0 being the center of the screen*/
    final double adjust = 0.234375;
    
    public PixyCam()
    {
        //0x54 is the defualt address for the Pixy
        wire = new I2C(Port.kOnboard, 0x54);

        //Sets the values for the PID and have it try to get to 0 which is the center of the PixyCam's field-of-View
        targetPID = new SRF_PID(P, I, D);
        targetPID.adjustPID(P, I, D);
        targetPID.setSetpoint(0);
        targetPID.setReverse(true);

        blocks[0] = new Block();
        blocks[1] = new Block();
        blocks[2] = new Block();
    }

    /*Reads the values from the pixy, determines if it has usable data, converts two 8-bit values to one 16-bit,
      and returns data as a block*/
    public void getBlock(){
        //values from buffer of the pixyCam
        byte[] buff = new byte[65];
        //used to cycle through bits in buff
        int counter = 0;
        //sets the limit for looking through buffer
        int endCounter = 17;
        //used to determine the number of bytes of usable data is in the block
        int numBytes = 0;
        int blockStart;

        boolean viableBlock = true;

        wire.read(0, 65, buff);

        //Goes through data and determines position of first sync word
        //If it can't find one sets pixyHasTarget to false
        while(counter < endCounter) {
            //System.out.println("in finding start start");
            if(buff[counter] == 85 && buff[counter+1] == -86){
                if((buff[counter+2] == 85 || buff[counter+2] == 0x86) && buff[counter+3] == -86) {
                    break;
                } else if(counter - 2 > 0) {
                    counter -= 2;
                    break;
                } 
            }
            counter++;
            //System.out.println("in finding start");
        }

        if(counter < endCounter) {
            blockStart = counter;

            for(int blocksIndex = 0; blocksIndex < 3; blocksIndex++) {
                counter = blockStart + blocksIndex*16;

                //Determines if the data has atleast 5 non-zero values
                //5 being 2 bits for sync word, 1 for signature, and 1 for width and height
                while(counter < endCounter) {
                    if(buff[counter] != 0)
                        numBytes++;
                    counter++;
                }
                if(numBytes < 5) {
                    viableBlock = false;
                }

                counter = blockStart + blocksIndex * 16;

                int sig = convert(buff[counter+6], buff[counter+7]);
                int y = convert(buff[counter+8], buff[counter+9]);
                int x = convert(buff[counter+10], buff[counter+11]);
                int width = convert(buff[counter+12], buff[counter+13]);
                int height = convert(buff[counter+14], buff[counter+15]);

                if(width <= 0 || height <= 0)
                    viableBlock = false;
                
                if(viableBlock) {
                    blocks[blocksIndex] = new Block(sig, y, x, width, height);
                    currentBlock = blocks[blocksIndex];
                } else {
                    blocks[blocksIndex] = new Block();
                }
                viableBlock = true;
                endCounter += 16;
            }

        } else {
            blocks[0] = new Block();
            blocks[1] = new Block();
            blocks[2] = new Block();
        }
    }

    public void displaySmartDashboard(){
        //Doesn't send data if PixyCam doesn't have target to avoid continuously sending the last target's data
        SmartDashboard.putBoolean("Pixy Target", pixyHasTarget);
        if(pixyHasTarget) {
            SmartDashboard.putNumber("Signature", currentBlock.getSignature());
            SmartDashboard.putNumber("X Coordinate",currentBlock.getX());
            SmartDashboard.putNumber("Y Coordinate",currentBlock.getY());
            SmartDashboard.putNumber("Width",currentBlock.getWidth());
            SmartDashboard.putNumber("Height",currentBlock.getHeight());
        }
    }

    public boolean pixyHasTarget(){
        return pixyHasTarget;
    }

    //takes in 2 bytes and turns them into one 16-bit value
    public int convert(Byte convert1, Byte convert2)
    {
        ByteBuffer converter = ByteBuffer.allocate(2);
        converter.put(convert2);
        converter.put(convert1);
        return converter.getShort(0);
    }

    /**
      If the Pixy has a target, Converts the X coordinate of the currrent block to be an angle out of the field-Of-View
      of the PixyCam. Then uses SRF_PID to generate a motor value between 1 to -1 intetended for the SRF_Swerve_Drive W value of Set().
      If Pixy doesn't have a target returns -2.
     */
    public double targeting()
    {
        getBlock();
        //System.out.println("past getBlock()");
        for(int i = 0; i < 3; i++) {
            if(blocks[i].getSignature() != -1) {
                //if(Math.abs(blocks[i].getHeight()/blocks[i].getWidth() - 1) < .2) {
                    currentBlock = blocks[i];
                    i = 3;
                    pixyHasTarget = true;
                //}
            }
            if(i==2)
                pixyHasTarget = false;
        }

        double output = -2;
        if(pixyHasTarget) {
            double targetPos = ((currentBlock.getX()-160)*adjust);
            SmartDashboard.putNumber(("targetPos"), targetPos);
            output = targetPID.computePID(targetPos, 0);
            SmartDashboard.putNumber("pidTarget" , output);
        }
        return output;
    }
}