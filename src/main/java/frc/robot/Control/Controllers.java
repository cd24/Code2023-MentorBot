package frc.robot.Control;

import edu.wpi.first.wpilibj2.command.Subsystem;

/// Provides a top-level object to contain the controllers available to
/// the system. In a typical match, this will entail one driver and one
/// operator.
public class Controllers implements Subsystem
{
    public Controller driverController;
    public Controller operatorController;

    public Controllers(Controller driverController, Controller operatorController) 
    {
        this.driverController = driverController;
        this.operatorController = operatorController;
    }

    // MARK: - Builders

    public Controllers withBoundIO() 
    {
        this.driverController.configure();
        this.operatorController.configure();
        return this;
    }

    // MARK: - Subsystem Overrides

    @Override
    public void periodic() 
    {
        // Driver actions come first...
        this.driverController.periodic();
        // Then we evaluate the operator controls
        this.operatorController.periodic();
    }
}