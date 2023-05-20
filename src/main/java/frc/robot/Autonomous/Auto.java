package frc.robot.Autonomous;

import frc.robot.subsystems.*;
import frc.robot.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveXMeters;
import frc.robot.commands.GridTrack;
import frc.robot.commands.TurnXDegrees;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/*
    Class to store autonomous sequences, including sequences such as intake
*/

public class Auto {
    public enum Selection {
        MOVEARM(1), MOVEWRIST(2), OTHER(3);
        public int val;
        private Selection(int val) {
            this.val = val;
        }
    }

    private RobotContainer robot;

    public Auto(RobotContainer robot) 
    {
        this.robot = robot;
    }

    public Command getShootCommand() { //Drive up and shoot
        return new SequentialCommandGroup(
            new GridTrack(robot.drivetrain, robot.limelight)
        );
    }

    public Command getBackupCommand() { //Back up and find new ball
        return new SequentialCommandGroup(
            new TurnXDegrees(
                180, 
                AutoConstants.TXDConstraints[0], 
                AutoConstants.TXDConstraints[1], 
                robot
            )
        );
    }

    public Command lowConeBackup() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> robot.wrist.setWristPositionAuto(Intake.ScorePos.STOW), robot.wrist),
            new InstantCommand(() -> robot.arm.setArmPositionAuto(Intake.ScorePos.STOW), robot.arm),
            new RunCommand(() -> robot.intake.set(-0.9), robot.intake).withTimeout(1),
            new WaitCommand(.5),
            new RunCommand(() -> robot.drivetrain.setOpenLoop(-0.25, -0.25), robot.drivetrain).withTimeout(2.0),
            new RunCommand(() -> robot.intake.set(0)).withTimeout(1)
        );
    }

    public Command highConeBackup() {
        return new SequentialCommandGroup(
            new RunCommand(() -> robot.drivetrain.setOpenLoop(-0.25, -0.25), robot.drivetrain).withTimeout(1.),
            new InstantCommand(() -> robot.wrist.setWristPositionAuto(Intake.ScorePos.MID), robot.wrist),
            new InstantCommand(() -> robot.arm.setArmPositionAuto(Intake.ScorePos.MID), robot.arm),
            new InstantCommand(() -> robot.drivetrain.setOpenLoop(0, 0), robot.drivetrain),
            new WaitCommand(2.5),
            new RunCommand(() -> robot.drivetrain.setOpenLoop(0.25, 0.25), robot.drivetrain).withTimeout(1.),
            new RunCommand(() -> robot.intake.set(-0.9), robot.intake).withTimeout(1),
            new RunCommand(() -> robot.drivetrain.setOpenLoop(-0.25, -0.25), robot.drivetrain).withTimeout(.4),
            new RunCommand(() -> robot.intake.set(0)).withTimeout(1),
            new WaitCommand(2.),
            new InstantCommand(() -> robot.wrist.setWristPositionAuto(Intake.ScorePos.STOW), robot.wrist),
            new InstantCommand(() -> robot.arm.setArmPositionAuto(Intake.ScorePos.STOW), robot.arm),
            new RunCommand(() -> robot.drivetrain.setOpenLoop(-0.25, -0.25), robot.drivetrain).withTimeout(1.0)

        );
    }
}