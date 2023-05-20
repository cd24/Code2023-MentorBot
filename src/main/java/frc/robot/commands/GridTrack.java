package frc.robot.commands;
//thank you william our lord and savior
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.CustomUtil.Timeframe;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class GridTrack implements Command {
    private static final PIDController TURN_PID_CONTROLLER = new PIDController(VisionConstants.kPTurn,
            VisionConstants.kITurn, VisionConstants.kDTurn);
    
    private Timeframe<Integer> timeframe;

    private Drivetrain drivetrain;
    private Limelight limelight;

    public GridTrack(Drivetrain drivetrain, Limelight limelight) {
        this.timeframe = new Timeframe<>(1.5, 1.0/Constants.dt);
        this.drivetrain = drivetrain;
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        timeframe.reset();

        limelight.setLEDMode(Limelight.LEDMode.ON);
        //we will only have 1 pipeline
        limelight.setPipeline(Limelight.IntakeVisionPipeline.ROBOT);
    }

    boolean atTarget;
    @Override
    public void execute() {
        double left, right;
        double turnError = limelight.getXOffset();

        SmartDashboard.putNumber("x off", limelight.getXOffset());
        SmartDashboard.putNumber("y off", limelight.getYOffset());

        double turn = TURN_PID_CONTROLLER.calculate(turnError, 0);
        SmartDashboard.putNumber("turn", turn);
        
        // Turns in place when there is no throttle input
        left = turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens;
        right = -turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens;

        left = drivetrain.FEEDFORWARD.calculate(left) / Constants.kMaxVoltage;
        right = drivetrain.FEEDFORWARD.calculate(right) / Constants.kMaxVoltage;
        atTarget = (int) Math.abs(turnError) <= VisionConstants.kTurnTolerance;
        if(atTarget) { //TODO: Test timeframe and if it works well, tune the desired "matching percentage"
            timeframe.update(1);
        } else {
            timeframe.update(0);
        }
        SmartDashboard.putNumber("AtTarget?", (atTarget) ? 1 : 0);
        SmartDashboard.putNumber("Mean Accuracy", (timeframe.getAverageValue()));
        drivetrain.setOpenLoop(left, right);

    }

    @Override
    public boolean isFinished() {
         return timeframe.percentEqual(1) >= 0.85;
    }

    @Override
    public void end(boolean interrupted) {
        limelight.setLEDMode(Limelight.LEDMode.ON);
        limelight.setPipeline(Limelight.IntakeVisionPipeline.DRIVER); //change
        drivetrain.setOpenLoop(0.0, 0.0);
        timeframe.reset();
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}