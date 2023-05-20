package frc.robot.control.schemes.competition;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveTajectoryControl {

    private Trajectory trajectory;
    private Drivetrain drivetrain;

    public DriveTajectoryControl(String file, Drivetrain drivetrain)
    {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(file);
        this.drivetrain = drivetrain;
        this.trajectory = new Trajectory();
        
        try {
            this.trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            System.out.println("PATHWEAVER NOT WORKING !!!!!! HEHEHHA");
            e.printStackTrace();
            DriverStation.reportError("PATH FAILED, CHECK PATH LOCATION", e.getStackTrace());
        }
    }

    public Command pathweaverCommand() 
    {
         // Create a voltage constraint to ensure we don't accelerate too fast
         var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
             drivetrain.FEEDFORWARD,
             drivetrain.KINEMATICS,
             10
        );

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    DrivetrainConstants.kMaxSpeedMPS/3,
                    DrivetrainConstants.kMaxAcceleration)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(drivetrain.KINEMATICS)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
        
        // An example trajectory to follow.  All units in meters.
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory, 
            (()-> { return drivetrain.ODOMETRY.getPoseMeters(); }),
            new RamseteController(),
            drivetrain.FEEDFORWARD,
            drivetrain.KINEMATICS,
            drivetrain::getWheelSpeeds,
            new PIDController(DrivetrainConstants.kPV, 0, 0),
            new PIDController(DrivetrainConstants.kPV, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::setVoltages,
            drivetrain
        );
        drivetrain.resetEncoders();
        // Reset odometry to the starting pose of the trajectory.
        drivetrain.ODOMETRY.resetPosition(
            drivetrain.navX.getRotation2d(), 
            drivetrain.getLeftEnc(), 
            drivetrain.getRightEnc(), 
            trajectory.getInitialPose()
            );

        // Run path following command, then stop at the end.
        return new InstantCommand(
            () -> drivetrain.ODOMETRY.resetPosition(
                    drivetrain.navX.getRotation2d(), 
                    drivetrain.getLeftEnc(), 
                    drivetrain.getRightEnc(), 
                    trajectory.getInitialPose()
                )
            ).andThen(
            ramseteCommand.andThen(
                () -> drivetrain.setOpenLoop(0, 0)
            )
        );
    }
}
