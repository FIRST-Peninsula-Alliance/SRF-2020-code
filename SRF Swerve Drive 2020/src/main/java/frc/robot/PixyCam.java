package frc.robot;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PixyCam {

    //An Object to hold the data from the pixy
    //Data: Signature(what block sent), X and Y coordinates of center of detected object, width, Height
    Block currentBlock;

    I2C wire;
    //Tells if the the we've recieved a valid block from the pixy camera
    Boolean pixyHasTarget;

    public PixyCam(){
        //0x54 is the defualt address for the Pixy
        wire = new I2C(Port.kOnboard, 0x54);
        currentBlock = new Block();
    }

    //Reads the values from the pixy, determines if it has usable data, converts two 8-bit values to one 16-bit,
    //and returns data as a block
    public Block getBlock(){
        byte[] buff = new byte[32];
        int i = 0;
        int numBytes = 0;

        wire.read(0, 32, buff);

        //Determines if the data has atleast 5 non-zero values
        //5 being 2 bits for sync word, 1 for signature, and 1 for width and height
        while(i < 17) {
            if(buff[i] != 0)
                numBytes++;
            i++;
        }
        if(numBytes < 5) {
            pixyHasTarget = false;
            return null;
        }
        
        //Goes through data and determines position of first sync word
        //If it can't find one returns null and sets pixyHasTarget to false
        i = 0;
        while(i < 17){
            if(buff[i] == 85 && buff[i+1] == -86){
               if((buff[i+2] == 85 || buff[i+2] == 0x86) && buff[i+3] == -86) {
                    break;
                } else {
                    i-= 2;
                    break;
                } 
            } else {
                i++;
            }
        }
        if(i >= 17) {
            pixyHasTarget = false;
            return null;
        }

        int sig = convert(buff[i+6], buff[i+7]);
        int y = convert(buff[i+8], buff[i+9]);
        int x = convert(buff[i+10], buff[i+11]);
        int width = convert(buff[i+12], buff[i+13]);
        int height = convert(buff[i+14], buff[i+15]);
        currentBlock.set(sig,x,y,width,height);

        pixyHasTarget = true;
        return currentBlock;
    }

    public void displaySmartDashboard(){
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
}