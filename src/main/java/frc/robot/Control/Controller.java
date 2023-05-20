package frc.robot.control;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

/// Describes an instance of a controller with multiple formats for 
/// binding controls.
///
/// This class takes as its input an XboxController instance, allowing
/// code higher up in the initialization stack to define which port 
/// it is bound to. 
///
/// This class conforms to ``Subsystem`` so it can receive ``periodic``
/// calls like any other subsystem. This allows the controller object 
/// to operate over a robot, as needed. This creates strong type bindings
/// to the specfic robot implementations. For an example, see ``DriverController.java``
public class Controller
{
    /// A reference to the xbox controller this controller instance 
    /// represents
    public XboxController controller;

    /// A shared reference to the robot, useful for binding values at
    /// configuration or during the periodic function.
    public RobotContainer robot;

    /// A convinience object for wrapping button triggers. This can be
    /// a simpler starting point for prototyping and testing behavior 
    /// while the periodic function is more robust for combining multiple
    /// buttons together
    public Buttons buttons;

    /// A convinience object for wrapping button triggers. This can be
    /// a simpler starting point for prototyping and testing behavior 
    /// while the periodic function is more robust for combining multiple
    /// buttons together
    public DPAD dpad;

    public Object leftBumper;

    public Controller(int controllerPort, RobotContainer robot) 
    {
        XboxController controller = new XboxController(controllerPort);
        this.controller = controller;
        this.buttons = new Buttons(controller);
        this.dpad = new DPAD(controller);
        this.robot = robot;
    }

    // MARK: - Configuration

    public void configure() {
        // Stub for subclasses to configure their bindings post initialization
    }

    // MARK: - Helper Math

    /**
     * Deadbands an input to [-1, -deadband], [deadband, 1], rescaling inputs to be
     * linear from (deadband, 0) to (1,1)
     * 
     * @param input    The input value to rescale
     * @param deadband The deadband
     * @return the input rescaled and to fit [-1, -deadband], [deadband, 1]
     */
    public double deadband(double input, double deadband) {
        if(Math.abs(input) <= deadband) {
            return 0;
        } else if(Math.abs(input) == 1) {
            return input;
        } else {
            return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
        }
    }

    // MARK: - Provide a periodic binding

    public void periodic() {
    }

    // MARK: - Reporting to Smart Dashboard

    public void reportToSmartDashboard() {
        this.reportJoystickValues();
        this.reportButtonState();
        this.reportBumperState();
    }

    public void reportJoystickValues() {
        SmartDashboard.putNumber("Left Joystick Rotation", this.controller.getLeftX());
        SmartDashboard.putNumber("Right Joystick Rotation", this.controller.getRightX());
    }

    public void reportButtonState() {
        SmartDashboard.putBoolean("A Button", this.controller.getAButton());
        SmartDashboard.putBoolean("A Button Pressed", this.controller.getAButtonPressed());

        SmartDashboard.putBoolean("B Button", this.controller.getBButton());
        SmartDashboard.putBoolean("B Button Pressed", this.controller.getBButtonPressed());

        SmartDashboard.putBoolean("Y Button", this.controller.getYButton());
        SmartDashboard.putBoolean("Y Button Pressed", this.controller.getYButtonPressed());

        SmartDashboard.putBoolean("X Button", this.controller.getXButton());
        SmartDashboard.putBoolean("X Button Pressed", this.controller.getXButtonPressed());

        SmartDashboard.putBoolean("Start Button", this.controller.getStartButton());
        SmartDashboard.putBoolean("Start Button Pressed", this.controller.getStartButtonPressed());

        SmartDashboard.putBoolean("Back Button", this.controller.getBackButton());
        SmartDashboard.putBoolean("Back Button Pressed", this.controller.getBackButtonPressed());

        SmartDashboard.putBoolean("Left Stick Button", this.controller.getLeftStickButton());
        SmartDashboard.putBoolean("Left Stick Button Pressed", this.controller.getLeftStickButtonPressed());

        SmartDashboard.putBoolean("Right Stick Button", this.controller.getRightStickButton());
        SmartDashboard.putBoolean("Right Stick Button Pressed", this.controller.getRightStickButtonPressed());
    }

    public void reportBumperState() {
        SmartDashboard.putBoolean("Left Bumper Button", this.controller.getLeftBumper());
        SmartDashboard.putBoolean("Left Bumper Button Pressed", this.controller.getLeftBumperPressed());

        SmartDashboard.putBoolean("Right Bumper Button", this.controller.getRightBumper());
        SmartDashboard.putBoolean("Right Bumper Button Pressed", this.controller.getRightBumperPressed());
    }
}