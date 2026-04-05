package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.spark.config.SparkMaxConfig;

/** Grabber. */
public class Grabber {

    private final int PIVOT_MOTOR_ID = 30;
    private final int INTAKE_MOTOR_ID = 31;

    // private final double PIVOT_ENCODER_CONVERSION_FACTOR = (20.0 / 64.0) * 360.0;

    private final double MAX_PIVOT_ANGLE = 132.5;
    private final double MIN_PIVOT_ANGLE = 2.25;

    private final double PIVOT_DOWN_VOLTAGE = -4;
    private final double PIVOT_UP_VOLTAGE = 5;
    private final double INTAKE_VOLTAGE = 4;

    private final double JOSTLE_MAX_DELAY_SECONDS = 0.1;
    private final double JOSTLE_DOWN_TIME_MULT = 2.25;
    private final double BASE_JOSTLE_TIME = 0.05;
    private final Timer jostleTimer = new Timer();
    private int jostleStep = 0;
    private double currentJostleDelay = 0;
    private double currJostleTime = BASE_JOSTLE_TIME;

    private SparkBase pivotMotor;
    private SparkBase intakeMotor;
    private SparkBaseConfig pivotMotorConfig;
    private SparkBaseConfig intakeMotorConfig;
    private AbsoluteEncoder pivotEncoder;
    private AbsoluteEncoderConfig pivotEncoderConfig;

    public Grabber() {
        pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        pivotEncoderConfig = new AbsoluteEncoderConfig();
        pivotEncoderConfig.inverted(true);
        pivotMotorConfig = new SparkMaxConfig();
        pivotMotorConfig
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(Robot.NEO_CURRENT_LIMIT)
            .disableFollowerMode()
            .apply(pivotEncoderConfig);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        intakeMotor = new SparkFlex(INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotorConfig = new SparkFlexConfig();
        intakeMotorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(Robot.VORTEX_CURRENT_LIMIT)
            .disableFollowerMode();
        intakeMotor.configure(intakeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public int raiseGrabber() {
        if (pivotEncoder.getPosition() < MAX_PIVOT_ANGLE) {
            pivotMotor.setVoltage(PIVOT_UP_VOLTAGE);
            // System.out.println("Raising grabber");
        } else {
            pivotMotor.stopMotor();
            // System.out.println("Grabber out of bounds on top side");
            return Robot.DONE;
        }

        return Robot.CONT;
    }

    public int lowerGrabber() {
        if (pivotEncoder.getPosition() > MIN_PIVOT_ANGLE) {
            pivotMotor.setVoltage(PIVOT_DOWN_VOLTAGE);
            // System.out.println("Lowering grabber");
        } else {
            pivotMotor.stopMotor();
            // System.out.println("Grabber out of bounds on bottom side");
            return Robot.DONE;
        }

        return Robot.CONT;
    }

    public void stopGrabber() {
        pivotMotor.stopMotor();
        // System.out.println("Nothing pressed");
    }

    // TODO: do this better for the new bot
    public int jostleGrabber() {
        if (jostleStep == 0) {
            jostleStep = 1;
            currentJostleDelay = Math.random() * JOSTLE_MAX_DELAY_SECONDS;
            // currJostleTime = BASE_JOSTLE_TIME;
            jostleTimer.restart();
        }

        int status = Robot.CONT;

        switch (jostleStep) {
            case 1:
                raiseGrabber();

                if (jostleTimer.hasElapsed(currJostleTime)) {
                    status = Robot.DONE;
                } else {
                    status = Robot.CONT;
                }
                break;
            case 2:
                lowerGrabber();

                if (jostleTimer.hasElapsed(currJostleTime * JOSTLE_DOWN_TIME_MULT)) {
                    status = Robot.DONE;
                } else {
                    status = Robot.CONT;
                }
                break;
            case 3:
                stopGrabber();

                if (jostleTimer.hasElapsed(currentJostleDelay)) {
                    status = Robot.DONE;
                } else {
                    status = Robot.CONT;
                }
                break;
            default:
                jostleStep = 0;
                currJostleTime += 0.01;
                pivotMotor.stopMotor();
                break;
        }
        
        if (status == Robot.DONE) {
            jostleStep++;
            jostleTimer.restart();
        }

        if (jostleStep >= 4) {
            return Robot.DONE;
        }

        return Robot.CONT;
    }

    public void resetJostle() {
        jostleStep = 0;
        currJostleTime = BASE_JOSTLE_TIME;
        // pivotEncoder.setPosition(0);
    }

    // public int autoLowerGrabber() {
    //     if (pivotEncoder.getPosition() > 0) {
    //         lowerGrabber();
    //         return Robot.CONT;
    //     } else {
    //         stopGrabber();
    //         return Robot.DONE;
    //     }
    // }

    public void intake() {
        intakeMotor.setVoltage(INTAKE_VOLTAGE);
    }

    public void outtake() {
        intakeMotor.setVoltage(-INTAKE_VOLTAGE);
    }

    public void stopWheel() {
        intakeMotor.stopMotor();
    }
}
