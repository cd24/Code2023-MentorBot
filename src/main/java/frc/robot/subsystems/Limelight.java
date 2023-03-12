package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants.*;
import frc.robot.Units;

public class Limelight {

    public String name;
    public NetworkTable limelight;

    public Limelight(String name) 
    {
        this.name = name;
        this.limelight = NetworkTableInstance.getDefault().getTable(name);
    }

    /**
     * Returns the vertical offset between the target and the crosshair in degrees
     * 
     * @return the vertical offset between the target and the crosshair in degrees
     */
    public double getYOffset() {
        return -this.limelight.getEntry("ty").getDouble(0.0);
    }

    public double getDistance() {
        double offsetAngle = this.limelight.getEntry("ty").getDouble(0.0);
        double angleGoalRads = (VisionConstants.mountAngle + offsetAngle) * (Math.PI/180);
        return Units.InchesToMeters(VisionConstants.goalHeightInches - VisionConstants.limelightHeightInches)/(Math.tan(angleGoalRads));
    }

    /**
     * Returns the horizontal offset between the target and the crosshair in degrees
     * 
     * @return the horizontal offset between the target and the crosshair in degrees
     */
    public double getXOffset() {
        return -this.limelight.getEntry("tx").getDouble(0);
    }

     /**
     * Set the LED mode on the Limelight
     * 
     * @param ledMode The mode to set the Limelight LEDs to
     */
    public void setLEDMode(LEDMode ledMode) {
        this.limelight.getEntry("ledMode").setNumber(ledMode.val);
    }

    /**
     * Sets the appearance of the Limelight camera stream
     * 
     * @param stream Stream mode to set the Limelight to
     */
    public void setStreamMode(StreamMode stream) {
        this.limelight.getEntry("stream").setNumber(stream.val);
    }

    /**
     * Sets Limelight vision pipeline
     * 
     * @param pipeline The pipeline to use
     */
    public void setPipeline(IntakeVisionPipeline pipeline) {
        this.limelight.getEntry("pipeline").setNumber(pipeline.val);
    }

    /**
 * Enum representing the different possible Limelight LED modes
 */
public enum LEDMode {
    PIPELINE(0), OFF(1), BLINK(2), ON(3);

    public int val;

    LEDMode(int val) {
        this.val = val;
    }
}

/**
 * Enum representing the different possible Limelight stream modes
 */
public enum StreamMode {
    SIDE_BY_SIDE(0), PIP_MAIN(1), PIP_SECONDARY(2);

    public int val;

    StreamMode(int val) {
        this.val = val;
    }
}

public enum IntakeVisionPipeline {
    RED(2), BLUE(1), ROBOT(0), DRIVER(3), INVALID(4);

    public int val;

    IntakeVisionPipeline(int val) {
        this.val = val;
    }
}

 /**
 * Enum representing the different possible Limelight vision pipelines
 */
public enum VisionPipeline {
    VISION(0), DRIVER(1);

    public int val;

    VisionPipeline(int val) {
        this.val = val;
    }
}
}
