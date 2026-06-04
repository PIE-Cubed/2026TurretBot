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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.config.SparkMaxConfig;

/** Grabber. */
public class Grabber {

    // CAN ID
    private final int PIVOT_MOTOR_ID = 30;
    private final int INTAKE_MOTOR_ID = 31;

    // Soft stop limits
    private final double MAX_PIVOT_ANGLE = 140.0;
    private final double MIN_PIVOT_ANGLE = 17.5;

    // Voltage settings
    private final double PIVOT_DOWN_VOLTAGE = -4;
    private final double PIVOT_UP_VOLTAGE = 5;
    private final double INTAKE_VOLTAGE = 6;

    // jostleGrabber() variables
    private final double JOSTLE_MAX_DELAY_SECONDS = 0.1;
    private final double JOSTLE_DOWN_TIME_MULT = 2.25;
    private final double BASE_JOSTLE_TIME = 0.1;
    private final Timer jostleTimer = new Timer();
    private int jostleStep = 0;
    private double currentJostleDelay = 0;
    private double currJostleTime = BASE_JOSTLE_TIME;

    // Motor variables
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
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(60)
            .disableFollowerMode()
            .apply(pivotEncoderConfig);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        intakeMotor = new SparkFlex(INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotorConfig = new SparkFlexConfig();
        intakeMotorConfig
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(60)
            .disableFollowerMode();
        intakeMotor.configure(intakeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * current logging
     */
    public void log() {
        SmartDashboard.putNumber("currents/grabber wheel current",  getInputCurrent(intakeMotor));
        SmartDashboard.putNumber("currents/grabber pivot current",  getInputCurrent(pivotMotor));

        Robot.totalCurrent += SmartDashboard.getNumber("currents/grabber wheel current",  getInputCurrent(intakeMotor));
        Robot.totalCurrent += SmartDashboard.getNumber("currents/grabber pivot current",  getInputCurrent(pivotMotor));
    }

    /**
     * helper function for current logging. extrapolates input current usage of a motor from the applied duty cycle and the stator current.
     * @param motor the motor to get the input current of
     * @return the input current of the motor
     */
    private double getInputCurrent(SparkBase motor) {
        // stator current * |duty cycle| = input current
        return motor.getOutputCurrent() * Math.abs(motor.getAppliedOutput());
    }

    /**
     * Raises the grabber up to the soft stop.
     * @return DONE if at soft stop, CONTINUE if not.
     */
    public int raiseGrabber() {
        // If current position is less than soft stop, apply voltage and return CONT.
        if (pivotEncoder.getPosition() < MAX_PIVOT_ANGLE) {
            pivotMotor.setVoltage(PIVOT_UP_VOLTAGE);
            // System.out.println("Raising grabber");
        } else {
            // If current position is above soft stop, stop motor and return DONE.
            pivotMotor.stopMotor();
            // System.out.println("Grabber out of bounds on top side");
            return Robot.DONE;
        }

        return Robot.CONT;
    }

    /**
     * Lowers the grabber down to the soft stop.
     * @return DONE if at soft stop, CONTINUE if not.
     */
    public int lowerGrabber() {
        // If current position is above soft stop, apply voltage and return CONT.
        if (pivotEncoder.getPosition() > MIN_PIVOT_ANGLE) {
            pivotMotor.setVoltage(PIVOT_DOWN_VOLTAGE);
            // System.out.println("Lowering grabber");
        } else {
            // If current position is lower than soft stop, stop motor and return DONE.
            pivotMotor.stopMotor();
            // System.out.println("Grabber out of bounds on bottom side");
            return Robot.DONE;
        }

        return Robot.CONT;
    }

    /**
     * Stops the pivot motor of the grabber.
     */
    public void stopGrabber() {
        pivotMotor.stopMotor();
        // System.out.println("Nothing pressed");
    }

    // TODO: do this better for the new bot
    /**
     * Moves the grabber in a jostling motion.
     * @return Status of the jostle. DONE if not in between motions, CONT if in the middle of moving the intake.
     */
    public int jostleGrabber() {
        // loop end/first time, reset code
        if (jostleStep == 0) {
            jostleStep = 1;
            currentJostleDelay = Math.random() * JOSTLE_MAX_DELAY_SECONDS;
            // currJostleTime = BASE_JOSTLE_TIME;
            jostleTimer.restart();
        }

        int status = Robot.CONT;

        switch (jostleStep) {
            case 1:
                // raise the grabber for the current jostle time
                raiseGrabber();

                if (jostleTimer.hasElapsed(currJostleTime)) {
                    status = Robot.DONE;
                } else {
                    status = Robot.CONT;
                }
                break;
            case 2:
                // lower grabber for the current jostle time times down time mult
                lowerGrabber();

                if (jostleTimer.hasElapsed(currJostleTime * JOSTLE_DOWN_TIME_MULT)) {
                    status = Robot.DONE;
                } else {
                    status = Robot.CONT;
                }
                break;
            case 3:
                // wait to raise again for the current jostle delay
                stopGrabber();

                if (jostleTimer.hasElapsed(currentJostleDelay)) {
                    status = Robot.DONE;
                } else {
                    status = Robot.CONT;
                }
                break;
            default:
                // raise current jostle time by 0.1 seconds every loop
                jostleStep = 0;
                currJostleTime += 0.1;
                pivotMotor.stopMotor();
                break;
        }
        
        if (status == Robot.DONE) {
            jostleStep++;
            jostleTimer.restart();
        }

        // return done if not in any state
        if (jostleStep >= 4) {
            return Robot.DONE;
        }

        return Robot.CONT;
    }

    /**
     * Resets the jostleGrabber() command.
     */
    public void resetJostle() {
        jostleStep = 0;
        currJostleTime = BASE_JOSTLE_TIME;
        // pivotEncoder.setPosition(0);
    }

    /**
     * Runs the intake roller in the normal direction.
     */
    public void intake() {
        intakeMotor.setVoltage(INTAKE_VOLTAGE);
    }

    /**
     * Runs the intake roller in the opposite direction.
     */
    public void outtake() {
        intakeMotor.setVoltage(-INTAKE_VOLTAGE);
    }

    /**
     * Stops the intake roller.
     */
    public void stopWheel() {
        intakeMotor.stopMotor();
    }
}
