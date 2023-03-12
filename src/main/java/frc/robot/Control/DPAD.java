package frc.robot.Control;

import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.XboxController;

public class DPAD 
{
    public final POVButton up;
    public final POVButton left;
    public final POVButton right;
    public final POVButton down;

    public DPAD(XboxController controller) 
    {
        this.up = new POVButton(controller, 0);
        this.right = new POVButton(controller, 90);
        this.down = new POVButton(controller, 180);
        this.left = new POVButton(controller, 270);
    }
}