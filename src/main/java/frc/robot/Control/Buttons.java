package frc.robot.Control;

import edu.wpi.first.wpilibj2.command.button.*; 
import edu.wpi.first.wpilibj.XboxController;

public class Buttons 
{
    public final Trigger a;
    public final Trigger b;
    public final Trigger y;
    public final Trigger x;
    public final Trigger lb;
    public final Trigger rb;
    public final Trigger view;
    public final Trigger menu;

    public Buttons(XboxController controller) 
    {
        this.a = new JoystickButton(controller, 1);
        this.b = new JoystickButton(controller, 2);
        this.x = new JoystickButton(controller, 3);
        this.y = new JoystickButton(controller, 4);
        this.lb = new JoystickButton(controller, 5);
        this.rb = new JoystickButton(controller, 6);
        this.view = new JoystickButton(controller, 7);
        this.menu = new JoystickButton(controller, 8);
    }
}