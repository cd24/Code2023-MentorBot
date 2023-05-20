// package frc.robot;

// import edu.wpi.first.wpilibj2.command.RepeatCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;

// import frc.robot.Controller;

// /// This is a top-level behavior object responsible for binding the
// /// initialized controller to robot behavior. This object controls
// /// the *driver* controller port.
// public class DriverController extends Controller {

//     @Override
//     public void configure() {
//         this.buttons.a
//             .whileTrue(new RepeatCommand(new RunCommand(() -> wrist.setWrist(0.1), this.robot.wrist)))
//             .onFalse(new RunCommand(() -> wrist.stopWrist(), this.robot.wrist));
//         this.buttons.b 
//             .onTrue(new RunCommand(() -> wrist.setWristPosition(0), this.robot.wrist));
//         this.buttons.x
//             .onTrue(new RunCommand(() -> wrist.setWristPosition(2.8), this.robot.wrist));
//     }
// }
