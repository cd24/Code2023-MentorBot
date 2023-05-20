
// package frc.robot;

// import frc.robot.ControlScheme;

// import edu.wpi.first.wpilibj.XboxController;

// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.POVButton;

// public class XboxControllerControlScheme implements ControlScheme 
// {
//     private JoystickButton 
//     // Control buttons
//         aButton, bButton, yButton, xButton, viewButton, menuButton,
//     // Bumper buttons
//         rbButton, lbButton, lButton, rButton;

//     // dpad
//     POVButton dpadUp, dpadRight, dpadDown, dpadLeft;

//     // Currently bound controller
//     XboxController controller;

//     // Currently bound robot
//     BobaBot robot;

//     public void configure(XboxController controller, BobaBot robot)
//     {
//         bindButtons(controller);
//         bindDpad(controller);
//         this.controller = controller;
//     }

//     double yAxisValue()
//     {
//         controller.getLeftY();
//     }

//     double xAxisValue()
//     {
//         controller.getRightX();
//     }

//     public void bindButtons(XboxController controller) 
//     {
//         aButton = new JoystickButton(controller, 1);
//         bButton = new JoystickButton(controller, 2);
//         xButton = new JoystickButton(controller, 3);
//         yButton = new JoystickButton(controller, 4);
//         lbButton = new JoystickButton(controller, 5);
//         rbButton = new JoystickButton(controller, 6);
//         viewButton = new JoystickButton(controller, 7);
//         menuButton = new JoystickButton(controller, 8);
//     }

//     public void bindDpad(XboxController controller) 
//     {
//         dpadUp = new POVButton(controller, 0);
//         dpadRight = new POVButton(controller, 90);
//         dpadDown = new POVButton(controller, 180);
//         dpadLeft = new POVButton(controller, 270);
//     }
// }
