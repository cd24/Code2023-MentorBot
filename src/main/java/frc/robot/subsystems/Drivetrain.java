package frc.robot.subsystems;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants;
import frc.robot.Units;
import frc.robot.Util;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain implements Subsystem {
    private final TalonFX leftLeader, leftFollower, rightLeader, rightFollower;
    
    public final List<TalonFX> motors;

    public final DifferentialDriveKinematics KINEMATICS;
    public final SimpleMotorFeedforward FEEDFORWARD;
    public final TrapezoidProfile.Constraints constraints;
    public final ProfiledPIDController LEFT_PID_CONTROLLER;
    public final ProfiledPIDController RIGHT_PID_CONTROLLER;
    public DifferentialDriveOdometry ODOMETRY;
    
    public AHRS navX;

    public Drivetrain(AHRS navX) {
        this.navX = navX;
        this.leftLeader = Util.createTalonFX(DrivetrainConstants.leftLeaderCAN);
        this.leftFollower = Util.createTalonFX(DrivetrainConstants.leftFollowerCAN);
        this.rightLeader = Util.createTalonFX(DrivetrainConstants.rightLeaderCAN);
        this.rightFollower = Util.createTalonFX(DrivetrainConstants.rightFollowerCAN);
        this.motors = List.of(leftLeader, leftFollower, rightLeader, rightFollower);

        this.KINEMATICS = new DifferentialDriveKinematics(DrivetrainConstants.kTrackWidth);
        this.FEEDFORWARD = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV, DrivetrainConstants.kA);
        this.constraints = new TrapezoidProfile.Constraints(DrivetrainConstants.kMaxSpeedMPS, DrivetrainConstants.kMaxAcceleration);
        this.LEFT_PID_CONTROLLER = new ProfiledPIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD, constraints);
        this.RIGHT_PID_CONTROLLER = new ProfiledPIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD, constraints);
        this.ODOMETRY = new DifferentialDriveOdometry(Rotation2d.fromDegrees(navX.getAngle()), getLeftEnc(), getRightEnc());

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
        //leftSlave.setNeutralMode(NeutralMode.Coast);
        //rightSlave.setNeutralMode(NeutralMode.Coast);
        // Inverting opposite sides of the drivetrain
        List.of(leftLeader , leftFollower).forEach(motor -> motor.setInverted(false));
        List.of(rightLeader , rightFollower).forEach(motor -> motor.setInverted(true));

        register();
    }

    @Override
    public void periodic() {
        ODOMETRY.update(Rotation2d.fromDegrees(-navX.getAngle()),
        getLeftEncMeters(),
        getRightEncMeters());
        SmartDashboard.putNumber("Left Master output: ", getLeftEncVelocityMeters());
        //SmartDashboard.putNumber("Left Slave output: ", leftSlave.getMotorOutputPercent());
        SmartDashboard.putNumber("Right Master output: ", getRightEncVelocityMeters());
        SmartDashboard.putNumber("NavX heading", navX.getAngle());
        //SmartDashboard.putNumber("Right Slave output: ", rightSlave.getMotorOutputPercent());
        
    }
    private int kInverted = 1; //1 or -1
    public int getkInvert() { //only for teleop driving, up to user to read this flag
        return kInverted;
    }

    public void setOpenLoop(double left, double right) {
        leftLeader.set(ControlMode.PercentOutput, left);
        rightLeader.set(ControlMode.PercentOutput, right);
    }

    public void setVoltages(double leftv, double rightv) {
        setOpenLoop(leftv/Constants.kMaxVoltage, rightv/Constants.kMaxVoltage);
    }

    public void setInverted(boolean status) { //For defense, the back of the robot becomes the front
        if(status) {
            kInverted = -1;
        } else {
            kInverted = 1;
        }
    }

    public void stop() {
        setOpenLoop(0, 0);
    }

    /**
     * Zeroes encoders
     */
    public void resetEncoders() {
        resetEncoders(0, 0);
    }
    
    /**
     * Sets encoders to a specific value
     * @param left  left wheel value
     * @param right right wheel value
     */
    public void resetEncoders(int left, int right) {
        rightLeader.setSelectedSensorPosition(right);
        leftLeader.setSelectedSensorPosition(left);
    }

    /**
     * @return the left and right drivetrain velocities (in meters/sec) as a DifferentialDriveWheelSpeeds object
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncVelocityMeters(), getRightEncVelocityMeters());
    }
    
    /**
     * @return the current position measurement of the left drivetrain encoder in talon native units (ticks)
     */
    public double getLeftEnc() {
        return leftLeader.getSelectedSensorPosition();
    }
    
    /**
     * @return the current position measurement of the right drivetrain encoder in talon native units (ticks/)
     */
    public double getRightEnc() {
        return rightLeader.getSelectedSensorPosition();
    }

    /**
     * @return the current position measurement of the left drivetrain encoder in meters
     */
    public double getLeftEncMeters() {
        return Units.DrivetrainUnits.TicksToMeters(getLeftEnc());
    }
    
    /**
     * @return the current position measurement of the right drivetrain encoder in meters
     */
    public double getRightEncMeters() {
        return Units.DrivetrainUnits.TicksToMeters(getRightEnc());
    }


    
    /**
     * @return the current velocity measurement of the left drivetrain encoder in talon native units (ticks/ds)
     */
    public double getLeftEncVelocity() {
        return leftLeader.getSelectedSensorVelocity();
    }
    
    /**
     * @return the current velocity measurement of the right drivetrain encoder in talon native units (ticks/ds)
     */
    public double getRightEncVelocity() {
        return rightLeader.getSelectedSensorVelocity();
    }

    /**
     * @return the current velocity measurement of the left drivetrain encoder in meters
     */
    public double getLeftEncVelocityMeters() {
        return Units.DrivetrainUnits.TicksPerDecisecondToMPS(getLeftEncVelocity());
    }

    /**
     * @return the current velocity measurement of the right drivetrain encoder in meters
     */
    public double getRightEncVelocityMeters() {
        return Units.DrivetrainUnits.TicksPerDecisecondToMPS(getRightEncVelocity());
    }
    
    /* Static class to contain the speeds of each side of the drivetrain */
    public class WheelState {
        public double left, right;
        
        public WheelState(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }
}