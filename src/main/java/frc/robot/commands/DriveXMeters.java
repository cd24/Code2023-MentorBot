package frc.robot.commands;

import java.util.Set;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Units.DrivetrainUnits;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivetrain;

public class DriveXMeters implements Command {

    private TrapezoidProfile.State goal;
    private TrapezoidProfile profile;
    private TrapezoidProfile.Constraints constraints;
    private double startTime;
    public enum Gear { 
        forwards, reverse;
    }
    private Gear gear;
    
    private Drivetrain drivetrain;

    /*
        @param distance     desired distance
        @param maxSpeedMPS  max speed during motion
        @param maxAccelMPSS max acceleration during motion
    */
    public DriveXMeters(double distance, double maxSpeedMPS, double maxAccelMPSS, Gear gear, Drivetrain drivetrain) {
        constraints = new TrapezoidProfile.Constraints(maxSpeedMPS, maxAccelMPSS);
        goal = new TrapezoidProfile.State(distance, 0.0);
        profile = new TrapezoidProfile(constraints, goal);
        
        this.gear = gear;
        this.drivetrain = drivetrain;
    }
    public DriveXMeters(double distance, double maxSpeedMPS, double maxAccelMPSS, Drivetrain drivetrain) {
        this(distance, maxSpeedMPS, maxAccelMPSS, Gear.forwards, drivetrain);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        //TrapezoidProfile.State profileCalc = profile.calculate(Constants.dt);
        double timestamp = Timer.getFPGATimestamp() - startTime;
        TrapezoidProfile.State profileCalc = profile.calculate(timestamp);
        double left, right;
        left = drivetrain.FEEDFORWARD.calculate(profileCalc.velocity);
        right = drivetrain.FEEDFORWARD.calculate(profileCalc.velocity);
        left += drivetrain.LEFT_PID_CONTROLLER.calculate(DrivetrainUnits.TicksToMeters(drivetrain.getLeftEnc()) , (gear == Gear.reverse) ? -profileCalc.position : profileCalc.position);
        right += drivetrain.RIGHT_PID_CONTROLLER.calculate(DrivetrainUnits.TicksToMeters(drivetrain.getRightEnc()), (gear == Gear.reverse) ? -profileCalc.position : profileCalc.position);
        left /= Constants.kMaxVoltage;
        right /= Constants.kMaxVoltage;
        if(gear == Gear.reverse) {
            left *= -1;
            right *= -1;
        }
        drivetrain.setOpenLoop(left, right);
        //profile = new TrapezoidProfile(constraints, goal, profileCalc);
    }

    @Override 
    public boolean isFinished() {
        return profile.isFinished(Timer.getFPGATimestamp() - startTime);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}