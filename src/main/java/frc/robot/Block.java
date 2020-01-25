package frc.robot;

public class Block
{
    private int signature; 
    private int xCenter;
    private int yCenter;
    private int width;
    private int height;

    public Block()
    {
        signature = -1;
        yCenter = -1;
        xCenter = -1;
        width = -1;
        height = -1;
    }

    public Block(int sig, int centerY, int centerX, int w, int h)
    {
        signature = sig;
        yCenter = centerY;
        xCenter = centerX;
        width = w;
        height = h;
    }

    public int getSignature(){
        return signature;
    }

    public int getY()
    {
        return yCenter;
    }

    public int getX()
    {
        return xCenter;
    }

    public int getHeight()
    {
        return height;
    }

    public int getWidth()
    {
        return width;
    }

    public void set(int sig, int centerY, int centerX, int w, int h)
    { 
        signature = sig;
        yCenter = centerY;
        xCenter = centerX;
        width = w;
        height = h;
    }
}