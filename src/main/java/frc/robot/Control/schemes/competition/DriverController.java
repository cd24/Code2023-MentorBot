package frc.robot.control.schemes.competition;

import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.RobotContainer;
import frc.robot.Constants.DriverConstants;
import frc.robot.control.Controller;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/// This is a top-level behavior object responsible for binding the
/// initialized controller to robot behavior. This object controls
/// the Driver controller port.
public class DriverController extends Controller {

    private Wrist wrist;
    private Drivetrain drivetrain;

    public DriverController(int controllerPort, RobotContainer robot) {
        super(controllerPort, robot);
        this.wrist = robot.wrist;
        this.drivetrain = robot.drivetrain;
    }

    @Override
    public void configure() {
        buttons.A
            .whileTrue(new RepeatCommand(new RunCommand(() -> wrist.setWrist(0.1), wrist)))
            .onFalse(new RunCommand(() -> wrist.stopWrist(), wrist));
        buttons.B 
            .onTrue(new RunCommand(() -> wrist.setWristPosition(0), wrist));
        buttons.X
            .onTrue(new RunCommand(() -> wrist.setWristPosition(2.8), wrist));
    }

    @Override
    public void periodic() {
        super.periodic();

        // Drivetrain control
        double throttle = -deadband(controller.getLeftY(), DriverConstants.kJoystickDeadband);
        double turn = deadband(controller.getRightX(), DriverConstants.kJoystickDeadband);

        if (throttle != 0) {
            turn *= drivetrain.getkInvert();
            throttle *= drivetrain.getkInvert();
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
        drivetrain.setOpenLoop(left, right); 
    }
}
