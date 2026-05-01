// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Logger;

/** Add your docs here. */
public class Turret {

    private SparkMax turretMotor;
    private SparkBaseConfig turretMotorConfig;
    
    private AbsoluteEncoder turretCRTEncoder1;
    private AbsoluteEncoderConfig turretCRTEncoder1Config;
    private DutyCycleEncoder turretCRTEncoder2;
    private RelativeEncoder turretEncoder;
    private EncoderConfig turretEncoderConfig;

    private PIDController turretPID;

    private final double MAX_TURRET_ANGLE_DEGREES = 365;

    private Transform2d turretPosOffset;

    private final double MODEL_STD_DEV       = 3.0;  // Q: model uncertainty
    private final double MEASUREMENT_STD_DEV = 0.35; // R: sensor noise
  
    // Identity system: x_next = x, y = x (angle passes straight through)
    private final LinearSystem<N1, N1, N1> plant = new LinearSystem<>(
        MatBuilder.fill(Nat.N1(), Nat.N1(), 1.0), // A: state transition (angle stays the same)
        MatBuilder.fill(Nat.N1(), Nat.N1(), 0.0), // B: no control input
        MatBuilder.fill(Nat.N1(), Nat.N1(), 1.0), // C: output = state
        MatBuilder.fill(Nat.N1(), Nat.N1(), 0.0)  // D: no feedthrough
    );
 
    private final KalmanFilter<N1, N1, N1> filter = new KalmanFilter<>(
        Nat.N1(),
        Nat.N1(),
        plant,
        VecBuilder.fill(MODEL_STD_DEV),
        VecBuilder.fill(MEASUREMENT_STD_DEV),
        0.20
    );

    private int inToleranceCount = 0;

    private double secondaryEncoderOffset = 0;

    public Turret(int turretMotorID, int secondaryEncoderChannel, double secondaryEncoderOffset,
                  Transform2d turretPosOffset, double TURRET_P, double TURRET_I, double TURRET_D, double TURRET_TOLERANCE) {
        this.turretPosOffset = turretPosOffset;

        turretMotor = new SparkMax(turretMotorID, MotorType.kBrushless);
        turretMotorConfig = new SparkMaxConfig()
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(25);
        
        turretEncoder = turretMotor.getEncoder();
        turretEncoderConfig = new EncoderConfig().positionConversionFactor(2.8921);

        turretCRTEncoder1 = turretMotor.getAbsoluteEncoder();
        turretCRTEncoder1Config = new AbsoluteEncoderConfig().inverted(true);
        turretMotorConfig.apply(turretCRTEncoder1Config);
        turretMotorConfig.apply(turretEncoderConfig);
        turretCRTEncoder2 = new DutyCycleEncoder(secondaryEncoderChannel);

        turretMotor.configure(turretMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        turretEncoder.setPosition(0);
        // crt = new TurretCRT(
        //     () -> Angle.ofBaseUnits(turretCRTEncoder1.getPosition(), Rotations), () -> Angle.ofBaseUnits(turretCRTEncoder2.get(), Rotations),
        //     0d, secondaryEncoderOffset, false, false);
        
        // crt.resolveAbsoluteAngle().ifPresent(angle -> {
        //     turretEncoder.setPosition(angle.in(Degrees));
        // });

        // SmartDashboard.putString("CRT Status", crt.getStatus());
        // SmartDashboard.putNumber("CRT Error (rot)", crt.getLastErrorRotations());
        
        turretPID = new PIDController(TURRET_P, TURRET_I, TURRET_D);
        turretPID.setTolerance(TURRET_TOLERANCE);

        this.secondaryEncoderOffset = secondaryEncoderOffset;
    }

    public double getMotorCurrent() {
        return getInputCurrent(turretMotor);
    }

    private double getInputCurrent(SparkBase motor) {
        return motor.getOutputCurrent() * Math.abs(motor.getAppliedOutput());
    }

    public void zeroEncoder() {
        turretEncoder.setPosition(0);
    }

    public void nudgeEncoder(double nudgeAmount) {
        turretEncoder.setPosition(turretEncoder.getPosition() + nudgeAmount);
    }

    public int pointAtWithVelocity(Pose2d targetPose, double inAirTime, Transform2d robotVel) {
        double x = robotVel.getX();
        double y = robotVel.getY();
        double o = robotVel.getRotation().getRadians();
        Logger.logStruct("targetPose2d " + turretMotor.getDeviceId(), targetPose.plus(getCurrentVelocity(x, y, o).times(-inAirTime)));
        Logger.logStruct("currentTurretVelocity " + turretMotor.getDeviceId(), getCurrentVelocity(x, y, o));
        SmartDashboard.putNumber("currentInAirTime " + turretMotor.getDeviceId(), inAirTime);
        return pointAt(targetPose.plus(getCurrentVelocity(x, y, o).times(-inAirTime)));
    }

    public int pointAt(Pose2d targetPose) {
        Pose2d currPose = Drive.getPose().plus(turretPosOffset);

        double dX = targetPose.getX() - currPose.getX();
        // SmartDashboard.putNumber(turretMotor.getDeviceId() + "dX", dX);
        double dY = targetPose.getY() - currPose.getY();
        // SmartDashboard.putNumber(turretMotor.getDeviceId() + "dY", dY);

        return setTargetFieldRotation(Math.toDegrees(Math.atan2(dY, dX)));
    }

    /**
     * -180 to 180
     * @param targetRotation
     * @return
     */
    public int setTargetFieldRotation(double targetRotation) {
        double currentRobotRotation = Drive.getPose().getRotation().getDegrees();

        filter.predict(VecBuilder.fill(0.0), 0.20); // No control input
        filter.correct(VecBuilder.fill(0.0), VecBuilder.fill(targetRotation));

        double filteredTargetAngle = filter.getXhat(0);
        Rotation2d targetAbsRotation = new Rotation2d(
            Units.degreesToRadians(filteredTargetAngle - currentRobotRotation)
        );
        // SmartDashboard.putNumber(turretMotor.getDeviceId() + "FieldRelativeTarget", targetRotation);
        // SmartDashboard.putNumber(turretMotor.getDeviceId() + "RobotRelativeTarget", targetAbsRotation.getDegrees());
        return setTargetFullRotation(targetAbsRotation.getDegrees() + 180);
    }

    // public void printSecondEncoderValue() {
    //     System.out.println(turretCRTEncoder1.getPosition());
    //     System.out.println(turretCRTEncoder2.get());
    //     System.out.println(turretMotor.getDeviceId());
    // }

    /**
     * 0 to 360
     * @param targetRotation
     * @return
     */
    public int setTargetAbsRotation(double targetRotation) {
        double currentPosition = turretEncoder.getPosition();
        targetRotation = MathUtil.clamp(targetRotation, 0, 360);

        if (targetRotation < MAX_TURRET_ANGLE_DEGREES - 360) {
            if (Math.abs((targetRotation + 360) - currentPosition) < Math.abs(currentPosition - targetRotation)) {
                targetRotation += 360;
            }
        }

        return setTargetFullRotation(targetRotation);
    }

    /**
     * 0 to whatever the max angle is
     * @param targetRotation
     * @return
     */
    public int setTargetFullRotation(double targetRotation) {
        double voltage = turretPID.calculate(turretEncoder.getPosition(), targetRotation);

        SmartDashboard.putNumber(turretMotor.getDeviceId() + "EncoderPosition", turretEncoder.getPosition());
        SmartDashboard.putNumber(turretMotor.getDeviceId() + "TargetPosition", targetRotation);

        turretMotor.setVoltage(voltage);

        if (turretPID.atSetpoint()) {
            inToleranceCount++;
        } else {
            inToleranceCount = 0;
        }

        if (inToleranceCount >= 5) {
            return Robot.DONE;
        }

        return Robot.CONT;
    }

    public void setTurretMotorVoltage(double voltage) {
        turretMotor.setVoltage(voltage);
    }

    public Transform2d getCurrentVelocity(double fwdPower, double leftPower, double ccwPower) {
        Rotation2d heading = Drive.getPose().getRotation();

        Transform2d vel = new Transform2d(
            fwdPower * SwerveModule.MAX_DRIVE_VEL_MPS, 
            leftPower * SwerveModule.MAX_DRIVE_VEL_MPS, 
            Rotation2d.fromDegrees(ccwPower * SwerveModule.MAX_ROTATE_VEL_DPS)
        );

        double omega = vel.getRotation().getRadians();

        // Rotate the robot-frame offset into field frame
        Translation2d fieldOffset = turretPosOffset.getTranslation().rotateBy(heading);

        Transform2d robotRelativeVelocity = new Transform2d(
            vel.getX() - omega * fieldOffset.getY(),
            vel.getY() + omega * fieldOffset.getX(),
            Rotation2d.kZero
        );

        return robotRelativeVelocity;
    }

    public double getAdjustedHubDistance(Transform2d robotVel) {
        double x = robotVel.getX();
        double y = robotVel.getY();
        double o = robotVel.getRotation().getRadians();

        return Drive.getPose().getTranslation().getDistance((
            AllianceUtil.isRedAlliance() 
            ? FieldConstants.hubRedAlliance.getTranslation() 
            : FieldConstants.hubBlueAlliance.getTranslation()
        ).minus(getCurrentVelocity(x, y, o).getTranslation()));
    }

    public void printEncoderValues() {
        SmartDashboard.putNumber("encoder1 " + turretMotor.getDeviceId(), turretCRTEncoder1.getPosition());
        SmartDashboard.putNumber("encoder2 " + turretMotor.getDeviceId(), turretCRTEncoder2.get() - secondaryEncoderOffset);
        SmartDashboard.putNumber("actualPos " + turretMotor.getDeviceId(), turretEncoder.getPosition());
    }
}
