package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {

    private SparkBase climbMotor;
    private SparkBaseConfig climbMotorConfig;

    // Motor IDs
    private final int CLIMB_MOTOR_ID = 51;

    private final double CLIMB_DOWN_VOLTAGE = -4.5;
    private final double CLIMB_UP_VOLTAGE = 4.5;

    private final double AUTO_CLIMB_LOW = 10;

    private RelativeEncoder climbEncoder;
    private EncoderConfig   climbEncoderConfig;


    public Climber() {
        climbMotor = new SparkMax(CLIMB_MOTOR_ID, MotorType.kBrushless);
        climbMotorConfig = new SparkMaxConfig();

        climbMotorConfig.idleMode(IdleMode.kBrake);
        climbMotorConfig.smartCurrentLimit(Robot.NEO_CURRENT_LIMIT);
        climbMotorConfig.inverted(false);

        climbEncoder = climbMotor.getEncoder();
        climbEncoder.setPosition(0.0);
        climbEncoderConfig = new EncoderConfig();
        climbEncoderConfig.positionConversionFactor(1.0);
        climbEncoderConfig.velocityConversionFactor(1.0);
        climbMotorConfig.apply(climbEncoderConfig); // Apply the encoder configuration to the SparkMax configuration

    
        climbMotor.configure(climbMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        // SmartDashboard.putNumber("Climb Encoder", 0.0);

    }
    // Starting at 0, 54 is a full extension
    public int moveClimberUp() {
        // SmartDashboard.putNumber("Climb Encoder", climbEncoder.getPosition());
        if (climbEncoder.getPosition() < 54) {
            climbMotor.setVoltage(CLIMB_UP_VOLTAGE);
            return Robot.CONT;
        } else {
            climbMotor.stopMotor();
            return Robot.DONE;
        }
    }

    public int moveClimberDown() {
        // SmartDashboard.putNumber("Climb Encoder", climbEncoder.getPosition());
        if (climbEncoder.getPosition() > 0) {
            climbMotor.setVoltage(CLIMB_DOWN_VOLTAGE);
            return Robot.CONT;
        } else {
            climbMotor.stopMotor();
            return Robot.DONE;
        }
    }

    public int autoClimberDown() {
        // SmartDashboard.putNumber("Climb Encoder", climbEncoder.getPosition());
        if (climbEncoder.getPosition() > AUTO_CLIMB_LOW) {
            climbMotor.setVoltage(CLIMB_DOWN_VOLTAGE);
            return Robot.CONT;
        } else {
            climbMotor.stopMotor();
            return Robot.DONE;
        }
    }

    public void stopClimberMotor() {
        climbMotor.stopMotor();
    }

    public void manualClimberUp() {
        // SmartDashboard.putNumber("Climb Encoder", climbEncoder.getPosition());
        climbMotor.setVoltage(CLIMB_UP_VOLTAGE);
    }

    public void manualClimberDown() {
        // SmartDashboard.putNumber("Climb Encoder", climbEncoder.getPosition());
        climbMotor.setVoltage(CLIMB_DOWN_VOLTAGE);
    }

    public void zeroClimberEncoder() {
        climbEncoder.setPosition(0);
    }

    /********************************************************************
     *
     * TEST PROGRAMS
     *
     ********************************************************************/

    public void testMotor(double upPower, double downPower) {
        if (upPower != 0) {
            climbMotor.set(upPower);
        } else if (downPower != 0) {
            climbMotor.set(downPower);
        } else {
            climbMotor.stopMotor();
        }
    }
}
