package frc.robot.subsystems;

import frc.robot.Util;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem {

    private final CANSparkMax intakeMotor;

    public Intake(int intakeCAN){
        this.intakeMotor = Util.createSparkMAX(intakeCAN, MotorType.kBrushless);
        register();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed", intakeMotor.get());
    }

    public void set(double value) {
        intakeMotor.set(value);
    }

    /**
     * Stops the intake
     */
    public void stopIntake() {
        intakeMotor.set(0);
    }
}
