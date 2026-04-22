package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;

/** a */
public class Hopper {
    private final double INDEX_POWER_VOLTS = 7;
    private final double KICKER_POWER_VOLTS = 9;

    private final int LEFT_KICKER_ID = 23;
    private final int RIGHT_KICKER_ID = 22;
    private final int LEFT_SPINDEXER_ID = 21;
    private final int RIGHT_SPINDEXER_ID = 20;

    private SparkBase rightSpindexerMotor = new SparkFlex(RIGHT_SPINDEXER_ID, MotorType.kBrushless);
    private SparkBaseConfig rightSpindexerMotorConfig = new SparkFlexConfig();

    private SparkBase leftSpindexerMotor = new SparkFlex(LEFT_SPINDEXER_ID, MotorType.kBrushless);
    private SparkBaseConfig leftSpindexerMotorConfig = new SparkFlexConfig();

    private SparkBase rightKickerMotor = new SparkFlex(RIGHT_KICKER_ID, MotorType.kBrushless);
    private SparkBaseConfig rightKickerMotorConfig = new SparkFlexConfig();

    private SparkBase leftKickerMotor = new SparkFlex(LEFT_KICKER_ID, MotorType.kBrushless);
    private SparkBaseConfig leftKickerMotorConfig = new SparkFlexConfig();

    public Hopper() {
        rightSpindexerMotorConfig
            .inverted(false)
            .smartCurrentLimit(55)
            .disableFollowerMode()
            .idleMode(IdleMode.kCoast)
            .secondaryCurrentLimit(75);

        rightSpindexerMotor.configure(
            rightSpindexerMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );

        leftSpindexerMotorConfig
            .inverted(true)
            .smartCurrentLimit(55)
            .disableFollowerMode()
            .idleMode(IdleMode.kCoast)
            .secondaryCurrentLimit(75);

        leftSpindexerMotor.configure(
            leftSpindexerMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );

        rightKickerMotorConfig
            .inverted(true)
            .smartCurrentLimit(80)
            .disableFollowerMode()
            .idleMode(IdleMode.kCoast)
            .secondaryCurrentLimit(100);

        rightKickerMotor.configure(
            rightKickerMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );

        leftKickerMotorConfig
            .inverted(false)
            .smartCurrentLimit(80)
            .disableFollowerMode()
            .idleMode(IdleMode.kCoast)
            .secondaryCurrentLimit(100);

        leftKickerMotor.configure(
            leftKickerMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void reverse() {
        rightSpindexerMotor.setVoltage(-INDEX_POWER_VOLTS * MathUtil.clamp(1.0 - Math.pow((rightSpindexerMotor.getMotorTemperature() - 45.0) / 45.0, 3), 0, 1));
        leftSpindexerMotor.setVoltage(-INDEX_POWER_VOLTS * MathUtil.clamp(1.0 - Math.pow((leftSpindexerMotor.getMotorTemperature() - 45.0) / 45.0, 3), 0, 1));
        rightKickerMotor.setVoltage(-KICKER_POWER_VOLTS * MathUtil.clamp(1.0 - Math.pow((rightKickerMotor.getMotorTemperature() - 45.0) / 45.0, 3), 0, 1));
        leftKickerMotor.setVoltage(-KICKER_POWER_VOLTS * MathUtil.clamp(1.0 - Math.pow((leftKickerMotor.getMotorTemperature() - 45.0) / 45.0, 3), 0, 1));
    }

    public void indexFuel() {
        rightSpindexerMotor.setVoltage(INDEX_POWER_VOLTS * MathUtil.clamp(1.0 - Math.pow((rightSpindexerMotor.getMotorTemperature() - 45.0) / 45.0, 3), 0, 1));
        leftSpindexerMotor.setVoltage(INDEX_POWER_VOLTS * MathUtil.clamp(1.0 - Math.pow((leftSpindexerMotor.getMotorTemperature() - 45.0) / 45.0, 3), 0, 1));
        rightKickerMotor.setVoltage(KICKER_POWER_VOLTS * MathUtil.clamp(1.0 - Math.pow((rightKickerMotor.getMotorTemperature() - 45.0) / 45.0, 3), 0, 1));
        leftKickerMotor.setVoltage(KICKER_POWER_VOLTS * MathUtil.clamp(1.0 - Math.pow((leftKickerMotor.getMotorTemperature() - 45.0) / 45.0, 3), 0, 1));
    }

    // public void indexRight() {
    //     // leftSpindexerMotor.setVoltage(-INDEX_POWER_VOLTS);
    //     rightSpindexerMotor.setVoltage(INDEX_POWER_VOLTS);
    //     rightKickerMotor.setVoltage(KICKER_POWER_VOLTS);
    // }

    // public void indexLeft() {
    //     // rightSpindexerMotor.setVoltage(-INDEX_POWER_VOLTS);
    //     leftSpindexerMotor.setVoltage(INDEX_POWER_VOLTS);
    //     leftKickerMotor.setVoltage(KICKER_POWER_VOLTS);
    // }

    // public void kickFuel() {
    //     kickLeft();
    //     kickRight();
    // }

    // public void kickLeft() {
    //     leftKickerMotor.setVoltage(KICKER_POWER_VOLTS);
    // }

    // public void kickRight() {
    //     rightKickerMotor.setVoltage(KICKER_POWER_VOLTS);
    // }

    public void stopMotors() {
        rightSpindexerMotor.stopMotor();
        leftSpindexerMotor.stopMotor();
        rightKickerMotor.stopMotor();
        leftKickerMotor.stopMotor();
    }
}
