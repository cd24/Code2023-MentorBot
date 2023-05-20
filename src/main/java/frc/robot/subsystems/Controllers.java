package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.control.Controller;
import frc.robot.control.schemes.competition.DriveTajectoryControl;

/// Provides a top-level object to contain the controllers available to
/// the system. In a typical match, this will entail one driver and one
/// operator.
public class Controllers implements Subsystem
{
    public Controller[] controllers;
    public DriveTajectoryControl trajectoryControl;

    public Controllers(DriveTajectoryControl trajectoryControl, Controller... controllers) 
    {
        this.controllers = controllers;
        this.trajectoryControl = trajectoryControl;
    }

    // MARK: - Builders

    public Controllers withBoundIO() 
    {
        for (Controller controller : controllers) {
            controller.configure();
        }
        return this;
    }

    // MARK: - Subsystem Overrides

    @Override
    public void periodic() 
    {
        for (Controller controller : controllers) {
            controller.periodic();
        }
    }

    public void reportToSmartDashboard() {
        for (Controller controller : controllers) {
            controller.reportToSmartDashboard();
        }
    }
}