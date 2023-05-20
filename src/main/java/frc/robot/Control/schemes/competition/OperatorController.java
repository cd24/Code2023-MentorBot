package frc.robot.control.schemes.competition;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.control.Controller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

/// This is a top-level behavior object responsible for binding the
/// initialized controller to robot behavior. This object controls
/// the *operator* controller port.
public class OperatorController extends Controller {

    private Wrist wrist;
    private Arm arm;
    private Intake intake;

    public OperatorController(int controllerPort, RobotContainer robot) {
        super(controllerPort, robot);
        this.wrist = robot.wrist;
        this.arm = robot.arm;
        this.intake = robot.intake;
    }

    @Override 
    public void configure() {
        // Cube Intake Positions
        buttons.Y
            .onTrue(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.HIGH), arm))
            .onTrue(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.HIGH), wrist))
            .onFalse(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.STOW), arm))
            .onFalse(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.STOW), wrist));

        buttons.A
            .onTrue(intakeCommandb())
            .onFalse(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.STOW), arm))
            .onFalse(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.STOW), wrist))
            .onFalse(new InstantCommand(() -> intake.running = false, intake));

        buttons.X
            .onTrue(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.MID), arm))
            .onTrue(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.MID), wrist))
            .onFalse(new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.STOW), arm))
            .onFalse(new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.STOW), wrist));

        buttons.B
            .onTrue(new RunCommand(() -> arm.setArmPosition(ArmConstants.kStow), arm))
            .onTrue(new RunCommand(() -> wrist.setWristPosition(WristConstants.kStow), wrist));

        // Cone Intake Positions
        dpad.up
            .whileTrue(new RunCommand(() -> wrist.setWrist(0.1), wrist))
            .onFalse(new RunCommand(() -> wrist.stopWrist(), wrist));

        dpad.right
            .whileTrue(new RunCommand(() -> arm.setOpenLoop(0.1), arm))
            .onFalse(new RunCommand(() -> arm.stopArm(), arm));

            //TODO not sure if this should be mid upright or mid sideways, see #programming
        dpad.left
            .whileTrue(new RunCommand(() -> arm.setOpenLoop(-0.1), arm))
            .onFalse(new RunCommand(() -> arm.stopArm(), arm));
            
        dpad.down
            .whileTrue(new RunCommand(() -> wrist.setWrist(-0.1), wrist))
            .onFalse(new RunCommand(() -> wrist.stopWrist(), wrist));

        // Cube Intake/Outtake Action
        buttons.rightBumper.onTrue(new InstantCommand(() -> intake.cone = false, intake));
            
        buttons.leftBumper.whileTrue(new InstantCommand(() -> intake.cone = true, intake));
    }

    @Override
    public void periodic() {
        super.periodic();

        // Arm Control
        double rightYDeadbanded = deadband(controller.getRightY(), DriverConstants.kJoystickDeadband);
        if (rightYDeadbanded > 0) {
            arm.setOpenLoop(0.1);
        } else if (rightYDeadbanded < 0) {
            arm.setOpenLoop(-0.1);
        } else {
            arm.stopArm();
        }

        // Wrist Control
        double wristValue = 0;
        double leftYDeadbanded = deadband(controller.getLeftY(), DriverConstants.kJoystickDeadband);
        if (leftYDeadbanded > 0) {
            wristValue = 0.1;
        } else if (leftYDeadbanded < 0) {
            wristValue = -0.1;
        }
        wrist.setWrist(wristValue);

        // Intake Control
        
        if (controller.getLeftTriggerAxis() > 0) {
            robot.intake.engage(controller.getRightTriggerAxis() > 0);
            robot.intake.isReleased = false;
        } else if (!robot.intake.isReleased) {
            robot.intake.set(0.);
            robot.intake.isReleased = true;
        }

        SmartDashboard.putBoolean("OPERATOR VIEW", buttons.view.getAsBoolean());

        
        if (intake.filterOutput() > 25.0) 
        {
            intake.dropSpeed();
        }
    }
    
    // MARK: - Intake Commands

    public Command intakeCommand() {
        // double sspeed = 0.;
        // if (Intake.cone) sspeed = 1.;
        // else sspeed = -1.;
        //double sspeed = (Intake.cone) ? 1. : -1.;
        
        intake.setCurrLimit(30);
        intake.running = true;
        intake.isReleased = true;
        //intake.set(speed);
        return new SequentialCommandGroup(
                new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.LOW), arm).withTimeout(0.15),
                new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.LOW), wrist)
            ).alongWith(new RunCommand(() -> intake.setAuto(1.), intake));
    }

    public Command intakeCommandb() {
        intake.setCurrLimit(30);
        intake.running = true;
        intake.isReleased = true;
        //intake.set(speed);
        return new SequentialCommandGroup(
                new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.LOW), arm).withTimeout(0.08),
                new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.LOW), arm).withTimeout(0.15),
                new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.LOW), wrist)
            ).alongWith(new RunCommand(() -> intake.setAuto(1.), intake));
    }

    public Command intakeCommandParallel() {
        intake.setCurrLimit(30);
        intake.running = true;
        intake.isReleased = true;
        //intake.set(speed);
        return new ParallelCommandGroup(
                new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.LOW), arm),
                new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.LOW), wrist),
                new RunCommand(() -> intake.setAuto(1.), intake)
            );
    }
    public Command setArmWrist(Intake.ScorePos pos) {
        return new RunCommand(() -> arm.setArmPositionAuto(pos), arm).alongWith(
               new RunCommand(() -> wrist.setWristPositionAuto(pos), wrist)
            );
    }

    public Command stow() {
        return new RunCommand(() -> arm.setArmPositionAuto(Intake.ScorePos.STOW), arm).alongWith(
               new RunCommand(() -> wrist.setWristPositionAuto(Intake.ScorePos.STOW), wrist)
            );
    }
}
