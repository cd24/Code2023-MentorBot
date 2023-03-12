package frc.robot.control.schemes.testing;

import frc.robot.RobotContainer;
import frc.robot.Constants.DriverConstants;
import frc.robot.control.Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/// This is an example controller scheme that could be used during testing to isolate 
/// debugging controller impact to this one class. See `Robot.java` for how this could
/// be used
public class IntakeIsolatedController extends Controller {

    public IntakeIsolatedController(int controllerPort, RobotContainer robot) {
        super(controllerPort, robot);
    }

    @Override
    public void periodic() {
        super.periodic();
        // Intake Control
        double LTvalue = deadband(controller.getLeftTriggerAxis(), DriverConstants.kJoystickDeadband);
        SmartDashboard.putNumber("LT Base Value", controller.getLeftTriggerAxis());
        SmartDashboard.putNumber("LT deadband Value", LTvalue);

        double RTvalue = deadband(controller.getRightTriggerAxis(), DriverConstants.kJoystickDeadband);
        SmartDashboard.putNumber("LT Base Value", controller.getRightTriggerAxis());
        SmartDashboard.putNumber("LT deadband Value", RTvalue);

        if(LTvalue > 0. && RTvalue == 0.){
            robot.intake.set(-0.75);
        } else if(LTvalue == 0 && RTvalue ==0){
            robot.intake.set(0.);
        }
        if(RTvalue > 0. && LTvalue == 0.){
            robot.intake.set(0.75);
        } else if (LTvalue==0 && RTvalue == 0){
            robot.intake.set(0.);
        }
    }
}
