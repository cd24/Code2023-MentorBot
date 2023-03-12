package frc.robot.subsystems;

import frc.robot.Util;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Wrist extends ProfiledPIDSubsystem {
    
    //private static CANSparkMax conveyorMotor;
    private final CANSparkMax wristMotor;
    private final RelativeEncoder wristEncoder;
    private final ArmFeedforward FEEDFORWARD;
    private SparkMaxPIDController pidController;
    // private RelativeEncoder relWristEncoder = wristMotor.getEncoder();
    private final SparkMaxAbsoluteEncoder wristAbsolulteEncoder;

    public Wrist(int wristMotorCAN) {
        super(
            new ProfiledPIDController(
                WristConstants.kP, 
                WristConstants.kI, 
                WristConstants.kD,
                new TrapezoidProfile.Constraints(WristConstants.kMaxVelocity, WristConstants.kMaxAcceleration)
            ),
            0
        );
        this.wristMotor = Util.createSparkMAX(wristMotorCAN, MotorType.kBrushless);
        this.wristMotor.setInverted(false);
        this.wristAbsolulteEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        this.wristEncoder = wristMotor.getEncoder();
        this.wristEncoder.setPosition(0);
        this.FEEDFORWARD = new ArmFeedforward(
            ArmConstants.kS, 
            ArmConstants.kCos, 
            ArmConstants.kV, 
            ArmConstants.kA
        );

        WristConstants.initialWristAngle = wristAbsolulteEncoder.getPosition();

        pidController = wristMotor.getPIDController();
        pidController.setP(WristConstants.kP);
        pidController.setI(WristConstants.kI);
        pidController.setD(WristConstants.kD);
        pidController.setIZone(0);
        pidController.setFF(0);
        pidController.setOutputRange(-0.3, 0.3);
        
        register();
    }

    /**
     * Sets the conveyor to spin at a percent of max speed
     * @param value Percent speed
     */
    /*public void setConveyor(double value) {
        conveyorMotor.set(value);
    }*/

    /**
     * Sets the intake to spin at a given voltage
     * 
     * @param value Percent of maximum voltage to send to motor
     */
    
    // add methods to spin in opposite direction
    public void setWrist(double value) {
        wristMotor.set(value);
    }
    

    public void stopWrist() {
        wristMotor.set(0);
    }

    public void periodic() {
        SmartDashboard.putNumber("WRIST: Current angle", wristEncoder.getPosition()*360.0/WristConstants.gearRatio);
        SmartDashboard.putNumber("WRIST: absolute angle", wristAbsolulteEncoder.getPosition()*36000.0/WristConstants.gearRatio);
        SmartDashboard.putNumber("WRIST: absolute position", wristAbsolulteEncoder.getPosition());
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        // TODO Auto-generated method stub
        double feedforward = FEEDFORWARD.calculate(setpoint.position, setpoint.velocity);
        wristMotor.setVoltage(output + feedforward);
        
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return wristEncoder.getPosition();
    }
    
    // public boolean getIntakeSensor() {
    //     if(!Robot.useV3()) {
    //         return !intakePhotoelectric.get();
    //     } else {
    //         return false; //(RobotContainer.colorSensorV3.getProximity() >= ConveyorConstants.minimumProximity);
    //     }
    // }

    public void setWristPosition(double position) {
        pidController.setReference(position - WristConstants.initialWristAngle, ControlType.kPosition);
        SmartDashboard.putNumber("SetWristPoint", position);
    }
}
