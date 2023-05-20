package frc.robot.subsystems;

import frc.robot.Util;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem {

    private final CANSparkMax intakeMotor;
    
    public LinearFilter filter;
    public double filterOutput;

    public boolean cone;
    public boolean running;
    public boolean isReleased;

    public Intake(int intakeCAN){
        this.intakeMotor = Util.createSparkMAX(intakeCAN, MotorType.kBrushless);
        this.filter = LinearFilter.movingAverage(30);
        this.cone = false;
        this.running = false;
        this.isReleased = false;
        this.filterOutput = 0;
        
        register();
    }

    @Override
    public void periodic() {

        // Report

        SmartDashboard.putNumber("Intake Speed", intakeMotor.get());

        SmartDashboard.putBoolean("CONE?", cone);
        SmartDashboard.putNumber("INTAKE CURRENT", intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("INTAKE VOLTAGE", intakeMotor.getAppliedOutput());
        SmartDashboard.putBoolean("INTAKE running", running);
        SmartDashboard.putNumber("INTAKE filtered output", filterOutput);
    }

    public void set(double value) {
        intakeMotor.set(value);
    }

    public void setCurrLimit(int limit) {
        intakeMotor.setSmartCurrentLimit(limit);
    }

    public void engage(boolean forwards) 
    {
        if (forwards) { 
            intakeMotor.setSmartCurrentLimit(30);
        }
        set(forwards ? 1 : -1);
        isReleased = false;
    }

    public void dropSpeed() 
    {
        intakeMotor.setSmartCurrentLimit(3);
        if (cone) set(.1);
        else set(-.1);
    }

    public double filterOutput() 
    {
        return filter.calculate(intakeMotor.getOutputCurrent());
    }

    public void setAuto(double value) {
        if (cone) set(value);
        else set(-value);
    }
    //hold methods periodically move the motor to avoid stalling (burnout) press button only once pls :>
    public void holdCone(double value) {
        Timer t = new Timer();
        t.start();
       
        if (t.get() == 2) {
            t.restart();
            intakeMotor.set(value);
            if (t.hasElapsed(1)) {
             stopIntake();
            }
        }
    }
    public void holdCube(double value) { // put in pos value
        value *=-1;
        Timer t = new Timer();
        t.start();
       
        if (t.get() % 2 == 0) {
            t.restart();
            intakeMotor.set(value);
            if (t.hasElapsed(1)) {
             stopIntake();
            }
        }
    }
    public void setHold(double value) {
        double current = intakeMotor.getOutputCurrent();
        intakeMotor.set(value);
        int limit = 40;
        if (current > limit) {
            intakeMotor.set(value/3.);
        } else {
            intakeMotor.set(value);
        }
       
    }
    /**
     * Stops the intake
     */
    public void stopIntake() {
        intakeMotor.set(0);
    }

    // MARK: - Support Types

    public enum ScorePos {
        HIGH(1), MID(2), LOW(3), STOW(4);
        public int val;
        private ScorePos(int val) {
            this.val = val;
        }
    }
}
