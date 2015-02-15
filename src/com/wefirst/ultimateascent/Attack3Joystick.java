package com.wefirst.ultimateascent;

import edu.wpi.first.wpilibj.Joystick;

 // A class to add extended methods specific to the Logitech Attack3 Joystick
public class Attack3Joystick extends Joystick{
    public Attack3Joystick(int port) {
        super(port);
    }

    protected Attack3Joystick(int port, int numAxisTypes, int numButtonTypes) {
        super(port, numAxisTypes, numButtonTypes);
    }
    
    // Returns the Z-axis as a number between 0 and 1
    public double getPower(){
        return (1 - ((this.getZ() + 1.0) / 2.0));
    }
}