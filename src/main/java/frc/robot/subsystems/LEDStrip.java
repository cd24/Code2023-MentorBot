package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDStrip {
    public final Spark sparkStrip;

    public LEDStrip(int identifier) {
        this.sparkStrip = new Spark(identifier);
    }

    public void enable() 
    {
        //color: Aqua 
        this.sparkStrip.set(0.81); 
    }

    public void disable() 
    {
        this.sparkStrip.set(0);
    }
}
