package frc.robot.Control;

import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.RobotContainer;
import frc.robot.Constants.*;

/// This is a top-level behavior object responsible for binding the
/// initialized controller to robot behavior. This object controls
/// the *operator* controller port.
public class OperatorController extends Controller {

    public OperatorController(int controllerPort, RobotContainer robot) {
        super(controllerPort, robot);
    }

    @Override 
    public void configure() {
        // Cube Intake Positions
        this.buttons.y
            .onTrue(new RunCommand(() -> robot.arm.setArmPosition(ArmConstants.kCubeHighScorePosition), robot.arm))
            .whileTrue(new RunCommand(() -> robot.wrist.setWristPosition(WristConstants.kCubeHighScorePosition), robot.wrist));
        this.buttons.a
            .onTrue(new RunCommand(() -> robot.arm.setArmPosition(ArmConstants.kCubeFloorIntakePosition), robot.arm))
            .onTrue(new RunCommand(() -> robot.wrist.setWristPosition(WristConstants.kCubeFloorIntakePosition), robot.wrist));
        this.buttons.x
            .onTrue(new RunCommand(() -> robot.arm.setArmPosition(ArmConstants.kCubeMidScorePosition), robot.arm))
            .onTrue(new RunCommand(() -> robot.wrist.setWristPosition(WristConstants.kCubeMidScorePosition), robot.wrist));
        this.buttons.b
            .onTrue(new RunCommand(() -> robot.arm.setArmPosition(ArmConstants.kStow), robot.arm))
            .onTrue(new RunCommand(() -> robot.wrist.setWristPosition(WristConstants.kStow), robot.wrist));

        // Cone Intake Positions
        this.dpad.up
            .onTrue(new RunCommand(() -> robot.arm.setArmPosition(ArmConstants.kConeHighUprightScorePosition), robot.arm))
            .onTrue(new RunCommand(() -> robot.wrist.setWristPosition(WristConstants.kConeHighUprightScorePosition), robot.wrist));
        this.dpad.right
            .onTrue(new RunCommand(() -> robot.arm.setArmPosition(ArmConstants.kConeMidUprightScorePosition), this.robot.arm))
            .onTrue(new RunCommand(() -> robot.wrist.setWristPosition(WristConstants.kConeMidUprightScorePosition), this.robot.wrist));
            //TODO not sure if this should be mid upright or mid sideways, see #programming
        this.dpad.left
            .onTrue(new RunCommand(() -> robot.arm.setArmPosition(ArmConstants.kConeFloorSidewaysIntakePosition), robot.arm))
            .onTrue(new RunCommand(() -> robot.wrist.setWristPosition(WristConstants.kConeFloorSidewaysIntakePosition), robot.wrist));
        this.dpad.down
            .onTrue(new RunCommand(() -> robot.arm.setArmPosition(ArmConstants.kConeFloorUprightIntakePosition), robot.arm))
            .onTrue (new RunCommand(() -> robot.wrist.setWristPosition(WristConstants.kConeFloorUprightIntakePosition), robot.wrist));

        // Cube Intake/Outtake Action
        this.buttons.rb
            .onTrue(new RunCommand(() -> robot.intake.set(.75), robot.wrist))
            .onFalse(new RunCommand(() -> robot.intake.set(0), robot.wrist));
        this.buttons.lb
            .onTrue(new RunCommand(() -> robot.intake.set(-.75), robot.wrist))
            .onFalse(new RunCommand(() -> robot.intake.set(0), robot.wrist));
    }

    @Override
    public void periodic() {
        super.periodic();

        // Arm Control
        double rightYDeadbanded = deadband(controller.getRightY(), DriverConstants.kJoystickDeadband);
        if (rightYDeadbanded > 0) {
            robot.arm.setOpenLoop(0.1);
        } else if (rightYDeadbanded < 0) {
            robot.arm.setOpenLoop(-0.1);
        } else {
            robot.arm.stopArm();
        }

        // Wrist Control
        double wristValue = 0;
        double leftYDeadbanded = deadband(controller.getLeftY(), DriverConstants.kJoystickDeadband);
        if (leftYDeadbanded > 0) {
            wristValue = 0.1;
        } else if (leftYDeadbanded < 0) {
            wristValue = -0.1;
        }
        robot.wrist.setWrist(wristValue);

        // Intake Control
        double LTvalue = deadband(controller.getLeftTriggerAxis(), DriverConstants.kJoystickDeadband);
        double RTvalue = deadband(controller.getRightTriggerAxis(), DriverConstants.kJoystickDeadband);
        if(LTvalue > 0. && RTvalue == 0.){
            robot.intake.set(-0.75);
        }else if(LTvalue == 0 && RTvalue ==0){
            robot.intake.set(0.);
        }
        if(RTvalue > 0. && LTvalue == 0.){
            robot.intake.set(0.75);
        }else if (LTvalue==0 && RTvalue == 0){
            robot.intake.set(0.);
        }
    }
}
