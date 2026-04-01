package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class Hopper {
    private final double INDEX_POWER_VOLTS = 5;
    private final double REVERSE_POWER_VOLTS = -3;

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
            .smartCurrentLimit(Robot.VORTEX_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast);

        rightSpindexerMotor.configure(
            rightSpindexerMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );

        leftSpindexerMotorConfig
            .inverted(true)
            .smartCurrentLimit(Robot.VORTEX_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast);

        leftSpindexerMotor.configure(
            leftSpindexerMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );

        rightKickerMotorConfig
            .inverted(true)
            .smartCurrentLimit(Robot.VORTEX_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast);

        rightKickerMotor.configure(
            rightKickerMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );

        leftKickerMotorConfig
            .inverted(false)
            .smartCurrentLimit(Robot.VORTEX_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast);

        leftKickerMotor.configure(
            leftKickerMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void indexFuel() {
        rightSpindexerMotor.setVoltage(INDEX_POWER_VOLTS);
        leftSpindexerMotor.setVoltage(INDEX_POWER_VOLTS);
        rightKickerMotor.setVoltage(INDEX_POWER_VOLTS);
        leftKickerMotor.setVoltage(INDEX_POWER_VOLTS);
    }

    public void reverseIndexer() {
        rightSpindexerMotor.setVoltage(REVERSE_POWER_VOLTS);
        leftSpindexerMotor.setVoltage(REVERSE_POWER_VOLTS);
        rightKickerMotor.setVoltage(REVERSE_POWER_VOLTS);
        leftKickerMotor.setVoltage(REVERSE_POWER_VOLTS);
    }

    public void stopMotors() {
        rightSpindexerMotor.stopMotor();
        leftSpindexerMotor.stopMotor();
        rightKickerMotor.stopMotor();
        leftKickerMotor.stopMotor();
    }
}
