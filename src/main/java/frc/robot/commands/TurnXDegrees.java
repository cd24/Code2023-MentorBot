package frc.robot.commands;

import java.util.Set;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class TurnXDegrees implements Command {

    private TrapezoidProfile.State goal;
    private TrapezoidProfile profile;
    private TrapezoidProfile.Constraints constraints;
    private PIDController turnController;
    private double startTime;

    public enum Gear { 
        forwards, reverse;
    }
    private double startAngle;

    // MARK: - Robot Components

    private AHRS navX;
    private Drivetrain drivetrain;

    /*
        @param distance     desired distance
        @param maxSpeedMPS  max speed during motion
        @param maxAccelMPSS max acceleration during motion
    */
    public TurnXDegrees(double degrees, double angVel, double angAcc, RobotContainer robot) {
        this.turnController = new PIDController(
            DrivetrainConstants.kP, 
            DrivetrainConstants.kI, 
            DrivetrainConstants.kD
        );
        degrees %= 360.;
        this.turnController.setTolerance(5);
        this.constraints = new TrapezoidProfile.Constraints(angVel, angAcc);
        this.goal = new TrapezoidProfile.State(degrees, 0.0);
        this.profile = new TrapezoidProfile(constraints, goal);
        this.navX = robot.navX;
        this.drivetrain = robot.drivetrain;
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        drivetrain.resetEncoders();
        startAngle = navX.getAngle();

    }

    @Override
    public void execute() {
        double timestamp = Timer.getFPGATimestamp() - startTime;
        TrapezoidProfile.State profileCalc = profile.calculate(timestamp);
        double left, right;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, profileCalc.velocity * Math.PI/180);
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
            DrivetrainConstants.kTrackWidth
        );
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        left = drivetrain.FEEDFORWARD.calculate(wheelSpeeds.leftMetersPerSecond);
        right = drivetrain.FEEDFORWARD.calculate(wheelSpeeds.rightMetersPerSecond);

        SmartDashboard.putNumber("leftwheel", left);
        SmartDashboard.putNumber("rightwheel", right);
        double turn = turnController.calculate(navX.getAngle() - startAngle, profileCalc.position);
        left += turn;
        right -= turn;
        left += drivetrain.LEFT_PID_CONTROLLER.calculate(
            navX.getAngle(), 
            profileCalc.position
        );
        right += drivetrain.RIGHT_PID_CONTROLLER.calculate(
            navX.getAngle(), 
            profileCalc.position
        );
        drivetrain.setOpenLoop(left/ Constants.kMaxVoltage, right / Constants.kMaxVoltage);
        SmartDashboard.putString("TurnXFinishedalt", "No");
        profile = new TrapezoidProfile(constraints, goal, profileCalc);
    }

    @Override 
    public boolean isFinished() {
        return profile.isFinished(Timer.getFPGATimestamp() - startTime);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("TurnXFinishedalt", "Yes");
        drivetrain.stop();
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
