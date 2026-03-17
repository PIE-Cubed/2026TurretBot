// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Made for REV 3in MAXSwerve modules. */
public class SwerveModule {

    private SparkFlex driveMotor;
    private SparkBaseConfig driveMotorConfig;

    private SparkMax rotateMotor;
    private SparkBaseConfig rotateMotorConfig;
    // Setting constant variables
    private final int DRIVE_MOTOR_CURRENT_LIMIT = 80;
    private final int ROTATE_MOTOR_CURRENT_LIMIT = 40;

    // Encoder creation
    private RelativeEncoder driveEncoder;
    private EncoderConfig driveEncoderConfig;

    private SparkAbsoluteEncoder absoluteEncoder;
    private AbsoluteEncoderConfig absoluteEncoderConfig;

    private PIDController rotatePIDController;

    public static ModuleConfig swerveModuleConfig;

    // PID
    private final double ROTATE_P = 0.007; //.00384
    private final double ROTATE_I = 0;
    private final double ROTATE_D = 0;
    private final double ROTATE_PID_TOLERANCE = 3; // In degrees
    private final double ROTATE_PID_TOLERANCE_LOOSE = 6;

    // Drive motor conversion factor cacluations
    public static final double EXTERNAL_GEARING = 5.08;
    public static final double WHEEL_DIAMETER_INCHES = 2.88;
    private final double WHEEL_CIRCUMFERENCE_FEET =
        (Math.PI * WHEEL_DIAMETER_INCHES) / 12; // Feet per rotation (circumference)
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(
        WHEEL_DIAMETER_INCHES
    );
    public static final double NEO_VORTEX_FREE_RPM = 6500.0 * 0.9; // multiply by 0.9 for practicality
    public static final double MAX_DRIVE_VEL_MPS =
        (((WHEEL_DIAMETER_METERS * Math.PI) / EXTERNAL_GEARING) * NEO_VORTEX_FREE_RPM) /
        60;
    // MAX_DRIVE_VEL_MPS is around 4.5 meters per second
    public static final double MAX_ROTATE_VEL_DPS = 280;
    // MAX_ROTATE_VEL_DPS is around 280 degrees per second
    public static final double REV_SPIKED_WHEEL_COF = 1.2;

    // Drive Motor Conversion Factors
    private final double DRIVE_POS_CONVERSION_FACTOR =
        WHEEL_CIRCUMFERENCE_FEET / EXTERNAL_GEARING; // Feet per tick
    private final double DRIVE_VEL_CONVERSION_FACTOR = DRIVE_POS_CONVERSION_FACTOR / 60.0; // Feet per second

    // Rotate Motor Conversion Factor
    private final double ROTATE_ENCODER_CONVERSION = 360; // Convert the rotate motor's encoder to degrees

    /**
     * <p> Creates ONE new swerve module. Create one of these for each swerve module on the robot.
     * @param driveID The CAN ID of the drive motor for the swerve module.
     * @param rotateID The CAN ID of the rotate motor for the swerve module.
     * @param invertDriveMotor Whether the drive motor should be inverted or not.
     */
    public SwerveModule(int driveID, int rotateID, boolean invertDriveMotor) {
        // Drive motor init and config
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        driveMotorConfig = new SparkFlexConfig();
        driveMotorConfig
            .smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake)
            .inverted(invertDriveMotor);

        // Rotate motor init and config
        rotateMotor = new SparkMax(rotateID, MotorType.kBrushless);
        rotateMotorConfig = new SparkMaxConfig();
        rotateMotorConfig
            .smartCurrentLimit(ROTATE_MOTOR_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);

        // Drive encoder configuration/setup
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPosition(0.0);

        driveEncoderConfig = new EncoderConfig();
        driveEncoderConfig
            .positionConversionFactor(Units.feetToMeters(DRIVE_POS_CONVERSION_FACTOR))
            .velocityConversionFactor(Units.feetToMeters(DRIVE_VEL_CONVERSION_FACTOR)); // DRIVE_VEL_CONVERSION_FACTOR
        driveMotorConfig.apply(driveEncoderConfig); // Apply the encoder configuration to the SparkMax configuration

        // Rotate encoder configuration/setup
        absoluteEncoder = rotateMotor.getAbsoluteEncoder();

        absoluteEncoderConfig = new AbsoluteEncoderConfig();
        absoluteEncoderConfig
            .positionConversionFactor(ROTATE_ENCODER_CONVERSION)
            .inverted(true);
        rotateMotorConfig.apply(absoluteEncoderConfig); // Apply the encoder configuration to the SparkMax configuration

        // Rotate PID controller configuration/setup
        rotatePIDController = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D);
        rotatePIDController.enableContinuousInput(0, 360);
        rotatePIDController.setTolerance(ROTATE_PID_TOLERANCE);
        rotatePIDController.setIntegratorRange(-0.1, 0.1);
        rotatePIDController.reset();

        // Apply the configurations(motor and encoder) to the SparkMax
        rotateMotor.configure(
            rotateMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );

        driveMotor.configure(
            driveMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );

        swerveModuleConfig = new ModuleConfig(
            WHEEL_DIAMETER_METERS / 2,
            MAX_DRIVE_VEL_MPS * 0.85,
            REV_SPIKED_WHEEL_COF * 0.9,
            DCMotor.getNeoVortex(1).withReduction(EXTERNAL_GEARING),
            Robot.VORTEX_CURRENT_LIMIT,
            1
        );
    }

    public void setDesiredState(SwerveModuleState swerveModuleState, boolean optimize) {
        // Optimizes the wheel movements
        /* When you optimize the swerve module state you minimize the wheel rotation.
         * For instance instead of rotating 180 degrees and driving forward you can
         * just drive in reverse and not rotate.  The optimized state will figure this
         * out for you.
         */
        if (optimize) {
            swerveModuleState.optimize(
                new Rotation2d(
                    MathUtil.angleModulus(
                        Units.degreesToRadians(absoluteEncoder.getPosition())
                    )
                )
            );
        }

        double currentAngleDegrees = absoluteEncoder.getPosition();
        double targetAngleDegrees = swerveModuleState.angle.getDegrees();
        double rotatePower = rotatePIDController.calculate(
            currentAngleDegrees,
            targetAngleDegrees
        );

        driveMotor.set(MathUtil.clamp(swerveModuleState.speedMetersPerSecond, -1.0, 1.0));
        rotateMotor.set(MathUtil.clamp(rotatePower, -1.0, 1.0));
    }

    public void setDriveMotorPower(double power) {
        driveMotor.set(MathUtil.clamp(power, -1, 1));
    }

    public void setRotateMotorPower(double power) {
        rotateMotor.set(MathUtil.clamp(power, -1, 1));
    }

    /// Returns SwerveModulePosition, with `distanceMeters` in feet
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            new Rotation2d(
                MathUtil.angleModulus(
                    Units.degreesToRadians(absoluteEncoder.getPosition())
                )
            )
        );
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            new Rotation2d(
                MathUtil.angleModulus(
                    Units.degreesToRadians(absoluteEncoder.getPosition())
                )
            )
        );
    }

    public void setLooseTolerance() {
        rotatePIDController.setTolerance(ROTATE_PID_TOLERANCE_LOOSE);
    }

    public void setTightTolerance() {
        rotatePIDController.setTolerance(ROTATE_PID_TOLERANCE);
    }

    public REVLibError resetPositionEncoder() {
        return driveEncoder.setPosition(0.0);
    }

    /**
     * <p>Set the drive motor smart current limit
     * <p>Performs no checks on currentLimit
     * @param currentLimit
     * @return REVLibError kOk on success
     */
    public REVLibError setDriveCurrentLimit(int currentLimit) {
        driveMotorConfig.smartCurrentLimit(currentLimit);

        return driveMotor.configure(
            driveMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /**
     * Checks if the rotate controller is at the setpoint.
     * @return atSetpoint
     */
    public boolean rotateControllerAtSetpoint() {
        return rotatePIDController.atSetpoint();
    }
}
