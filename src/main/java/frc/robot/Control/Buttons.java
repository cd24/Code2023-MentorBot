package frc.robot.control;

import edu.wpi.first.wpilibj2.command.button.*; 
import edu.wpi.first.wpilibj.XboxController;

public class Buttons 
{
    public final Trigger A;
    public final Trigger B;
    public final Trigger Y;
    public final Trigger X;
    public final Trigger leftBumper;
    public final Trigger rightBumper;
    public final Trigger view;
    public final Trigger menu;

    public Buttons(XboxController controller) 
    {
        this.A = new JoystickButton(controller, 1);
        this.B = new JoystickButton(controller, 2);
        this.X = new JoystickButton(controller, 3);
        this.Y = new JoystickButton(controller, 4);
        this.leftBumper = new JoystickButton(controller, 5);
        this.rightBumper = new JoystickButton(controller, 6);
        this.view = new JoystickButton(controller, 7);
        this.menu = new JoystickButton(controller, 8);
    }
}