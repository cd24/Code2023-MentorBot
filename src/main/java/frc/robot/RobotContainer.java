package frc.robot;

import frc.robot.Autonomous.Auto;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


import com.kauailabs.navx.frc.AHRS;

public class RobotContainer {

    // Motion
    public Drivetrain drivetrain;
    public Intake intake;
    public Wrist wrist;
    public Arm arm;

    // Vision
    public Limelight limelight;
    public AHRS navX; 
    
    // Style
    public LEDStrip ledStrip;

    public RobotContainer(
        Drivetrain drivetrain,
        Intake intake,
        Wrist wrist,
        Arm arm
    ) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.wrist = wrist;
        this.arm = arm;
        this.limelight = new Limelight("limelight-intake");
        this.ledStrip = new LEDStrip(0);
    }

    public void engage() {
        limelight.setLEDMode(Limelight.LEDMode.ON);

        drivetrain.resetEncoders();
    }

    public Command getAutonomousCommand(Auto.Selection selectedAuto) { //TODO: change auto based on selected strategy
        Command auto = null;
        switch (selectedAuto) {
            case MOVEARM:
                auto = Commands.runOnce(
                    () -> {
                        this.arm.setGoal(ArmConstants.autoDisplacementRads);
                        this.arm.enable();
                    }, 
                    this.arm
                );
                break;
            case MOVEWRIST:
                auto = Commands.runOnce(
                    () -> {
                        this.wrist.setGoal(WristConstants.autoDisplacementRads);
                        this.wrist.enable();
                    }, 
                    this.wrist
                );
                break;
            default:
                break;
        }
        return auto;
    }
}