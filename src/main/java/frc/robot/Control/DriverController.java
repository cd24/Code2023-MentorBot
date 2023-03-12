package frc.robot.Control;

import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.RobotContainer;
import frc.robot.Constants.DriverConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/// This is a top-level behavior object responsible for binding the
/// initialized controller to robot behavior. This object controls
/// the Driver controller port.
public class DriverController extends Controller {

    public DriverController(int controllerPort, RobotContainer robot) {
        super(controllerPort, robot);
    }

    @Override
    public void configure() {
        this.buttons.a
            .whileTrue(new RepeatCommand(new RunCommand(() -> this.robot.wrist.setWrist(0.1), this.robot.wrist)))
            .onFalse(new RunCommand(() -> robot.wrist.stopWrist(), robot.wrist));
        this.buttons.b 
            .onTrue(new RunCommand(() -> robot.wrist.setWristPosition(0), robot.wrist));
        this.buttons.x
            .onTrue(new RunCommand(() -> robot.wrist.setWristPosition(2.8), robot.wrist));
    }

    @Override
    public void periodic() {

        // Drivetrain control
        double throttle = -deadband(controller.getLeftY(), DriverConstants.kJoystickDeadband);
        double turn = deadband(controller.getRightX(), DriverConstants.kJoystickDeadband);

        if (throttle != 0) {
            turn *= robot.drivetrain.getkInvert();
            throttle *= robot.drivetrain.getkInvert();
        }
        
        SmartDashboard.putNumber("turn input", turn);
        SmartDashboard.putNumber("throttle input", throttle);

        double left, right;
         // Differential drive as long as throttle is greater than zero (deadbanded).
         if (throttle != 0) {
            left = (throttle + throttle * turn * DriverConstants.kTurnSens) * DriverConstants.kDriveSens;
            right = (throttle - throttle * turn * DriverConstants.kTurnSens) * DriverConstants.kDriveSens;

            // Normalize
            double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
            
            if(maxMagnitude > DriverConstants.kDriveSens) {
                left = left / maxMagnitude * DriverConstants.kDriveSens;
                right = right / maxMagnitude * DriverConstants.kDriveSens;
            } 
        } else {
            // Turns in place when there is no throttle input
            left = turn * DriverConstants.kTurnInPlaceSens;
            right = -turn * DriverConstants.kTurnInPlaceSens;
        }
        SmartDashboard.putNumber("left dt commanded", left);
        SmartDashboard.putNumber("right dt commanded", right);
        robot.drivetrain.setOpenLoop(left, right); 
    }
}
