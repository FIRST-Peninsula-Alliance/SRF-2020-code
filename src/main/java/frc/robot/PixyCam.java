package frc.robot;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class PixyCam {

    Block currentBlock;
    I2C wire;
    Boolean pixyHasTarget;

    public PixyCam(){
        //0x54 is the defualt address for the Pixy
        wire = new I2C(Port.kOnboard, 0x54);
        currentBlock = new Block();
    }

    public Block getBlock(){
        byte[] buff = new byte[32];
        int i = 0;
        int numBytes = 0;

        wire.read(0, 32, buff);

        while(i < 17) {
            if(buff[i] != 0)
                numBytes++;
            i++;
        }

        if(numBytes < 5) {
            pixyHasTarget = false;
            return null;
        }
        
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

    public boolean pixyHasTarget(){
        return pixyHasTarget;
    }

    public int convert(Byte convert1, Byte convert2)
    {
        ByteBuffer converter = ByteBuffer.allocate(2);
        converter.put(convert2);
        converter.put(convert1);
        return converter.getShort(0);
    }
}