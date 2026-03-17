package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class Hopper {

    private final int INDEX_MOTOR_ID = 33;
    private final int BELT_MOTOR_ID = 32;

    private SparkBase indexMotor = new SparkMax(INDEX_MOTOR_ID, MotorType.kBrushless);
    private SparkBaseConfig indexMotorConfig = new SparkMaxConfig();

    private SparkBase beltMotor = new SparkMax(BELT_MOTOR_ID, MotorType.kBrushless);
    private SparkBaseConfig beltMotorConfig = new SparkMaxConfig();

    public Hopper() {
        indexMotorConfig
            .inverted(true)
            .smartCurrentLimit(Robot.NEO_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);

        indexMotor.configure(
            indexMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );

        beltMotorConfig
            .smartCurrentLimit(Robot.NEO_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake)
            .follow(INDEX_MOTOR_ID, true);
        //    .inverted(false);

        beltMotor.configure(
            beltMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void indexFuel() {
        indexMotor.setVoltage(12);
    }

    public void reverseIndexer() {
        indexMotor.setVoltage(-12);
    }

    public void stopMotors() {
        indexMotor.stopMotor();
    }
}
