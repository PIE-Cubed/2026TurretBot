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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Logger;

/** Add your docs here. */
public class Turret {

    // Motors and encoders
    private SparkMax turretMotor;
    private SparkBaseConfig turretMotorConfig;
    
    private AbsoluteEncoder turretCRTEncoder1;
    private AbsoluteEncoderConfig turretCRTEncoder1Config;
    private DutyCycleEncoder turretCRTEncoder2;
    private RelativeEncoder turretEncoder;
    private EncoderConfig turretEncoderConfig;

    // Turret PID
    private PIDController turretPID;

    // Pose offset of the turret from the center of the robot.
    private Transform2d turretPosOffset;

    // private final double MAX_TURRET_ANGLE_DEGREES = 365;

    // Kalman filter variables
    private final double MODEL_STD_DEV       = 3.0;  // Q: model uncertainty
    private final double MEASUREMENT_STD_DEV = 0.35; // R: sensor noise
  
    private final LinearSystem<N1, N1, N1> plant = new LinearSystem<>(
        MatBuilder.fill(Nat.N1(), Nat.N1(), 1.0),
        MatBuilder.fill(Nat.N1(), Nat.N1(), 0.0),
        MatBuilder.fill(Nat.N1(), Nat.N1(), 1.0),
        MatBuilder.fill(Nat.N1(), Nat.N1(), 0.0)
    );
 
    private final KalmanFilter<N1, N1, N1> filter = new KalmanFilter<>(
        Nat.N1(),
        Nat.N1(),
        plant,
        VecBuilder.fill(MODEL_STD_DEV),
        VecBuilder.fill(MEASUREMENT_STD_DEV),
        0.20
    );

    // PID tolerance count to account for overshoot
    private int inToleranceCount = 0;

    // Used for CRT
    // private double secondaryEncoderOffset = 0;

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

        // zero relative encoder
        turretEncoder.setPosition(-180);

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

        // this.secondaryEncoderOffset = secondaryEncoderOffset;
    }

    /**
     * Gets the input current of the turret motor.
     * @return The input current of the turret motor.
     */
    public double getMotorCurrent() {
        return getInputCurrent(turretMotor);
    }

    /**
     * Helper function for getMotorCurrent() in order to get the input current from the Spark MAX
     * @param motor The motor to get the input current from.
     * @return The input current of the motor.
     */
    private double getInputCurrent(SparkBase motor) {
        return motor.getOutputCurrent() * Math.abs(motor.getAppliedOutput());
    }

    /**
     * Zeros the turret encoder. For use at the beginning of auton.
     */
    public void zeroEncoder() {
        turretEncoder.setPosition(0);
    }

    /**
     * Adds the passed double to the current position of the turret encoder.
     * Example use: The turret was zeroed 360 degrees too far, so you call this to nudge the encoder 360 degrees and fix the encoder.
     * @param nudgeAmount The amount to nudge the encoder by.
     */
    public void nudgeEncoder(double nudgeAmount) {
        turretEncoder.setPosition(turretEncoder.getPosition() + nudgeAmount);
    }

    /**
     * Points the turret at a target position on the field, accounting for robot velocity.
     * @param targetPose The target position.
     * @param inAirTime The amount of time the fuel is in the air for.
     * @param chassisPower Current x, y, and rotational duty cycle power being applied to the chassis.
     * @return Status of PID.
     */
    public int pointAtWithVelocity(Pose2d targetPose, double inAirTime, Transform2d chassisPower) {
        // get components from transform2d
        double xPower = chassisPower.getX();
        double yPower = chassisPower.getY();
        double rotationalPower = chassisPower.getRotation().getRadians();

        // logging values
        Logger.logStruct("targetPose2d " + turretMotor.getDeviceId(), targetPose.plus(getCurrentVelocity(xPower, yPower, rotationalPower).times(-inAirTime)));
        Logger.logStruct("currentTurretVelocity " + turretMotor.getDeviceId(), getCurrentVelocity(xPower, yPower, rotationalPower));
        SmartDashboard.putNumber("currentInAirTime " + turretMotor.getDeviceId(), inAirTime);

        // return status of PID function 
        return pointAt(targetPose.plus(getCurrentVelocity(xPower, yPower, rotationalPower).times(-inAirTime)));
    }

    /**
     * Points the turret at a target's location on the field.
     * @param targetPose The target location to point at.
     * @return Status of PID.
     */
    public int pointAt(Pose2d targetPose) {
        // get current turret pose
        Pose2d currTurretPose = Drive.getPose().plus(turretPosOffset);

        // get distance to target pose
        double dX = targetPose.getX() - currTurretPose.getX();
        double dY = targetPose.getY() - currTurretPose.getY();

        // return status of PID function using arctan of the distance to our target
        return setTargetFieldRotation(Math.toDegrees(Math.atan2(dY, dX)));
    }

    /**
     * Sets the target rotation of the turret based on a field relative rotation value.
     * @param targetRotation The target rotation.
     * @return Status of PID.
     */
    public int setTargetFieldRotation(double targetRotation) {
        // get current rotation on field
        Rotation2d currentRobotAngle = Drive.getPose().getRotation();

        // kalman filter
        filter.predict(VecBuilder.fill(0.0), 0.20); // No control input
        filter.correct(VecBuilder.fill(0.0), VecBuilder.fill(targetRotation));

        // get filter output
        Rotation2d filteredTargetAngle = Rotation2d.fromDegrees(filter.getXhat(0));

        // adjust to robot relative
        // TODO: fix this math
        Rotation2d targetAbsRotation = filteredTargetAngle.minus(currentRobotAngle);

        // logging
        SmartDashboard.putNumber(turretMotor.getDeviceId() + "FieldRelativeTarget", targetRotation);
        SmartDashboard.putNumber(turretMotor.getDeviceId() + "RobotRelativeTarget", targetAbsRotation.getDegrees());

        // return status of PID function
        return setTargetFullRotation(targetAbsRotation.getDegrees());
    }
    
    /**
     * PID control of the turret motor based on the passed target rotation.
     * @param targetRotation The target turret rotation from -180 to 180, in degrees.
     * @return Status of PID.
     */
    public int setTargetFullRotation(double targetRotation) {
        // modulus the rotation in case it is out of bounds.
        targetRotation = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(targetRotation)));

        // logging
        SmartDashboard.putNumber(turretMotor.getDeviceId() + "EncoderPosition", turretEncoder.getPosition());
        SmartDashboard.putNumber(turretMotor.getDeviceId() + "TargetPosition", targetRotation);

        // calculate PID voltage
        double voltage = turretPID.calculate(turretEncoder.getPosition(), targetRotation);

        // apply voltage
        turretMotor.setVoltage(voltage);

        // if at setpoint, increase the within tolerance count by 1
        if (turretPID.atSetpoint()) {
            inToleranceCount++;
        } else {
            // if not at setpoint, reset the within tolerance count
            inToleranceCount = 0;
        }

        // if this is the 5th or higher time that the turret is within tolerance, return status of DONE.
        if (inToleranceCount >= 5) {
            return Robot.DONE;
        }

        // not within tolerance, return status of CONTINUE.
        return Robot.CONT;
    }

    /*
     * below function kept in case we decide to go back to >360 degrees of rotation
     */

    /**
     * 0 to 360
     * @param targetRotation
     * @return
     */
    // public int setTargetAbsRotation(double targetRotation) {
    //     double currentPosition = turretEncoder.getPosition();
    //     targetRotation = MathUtil.clamp(targetRotation, 0, 360);

    //     if (targetRotation < MAX_TURRET_ANGLE_DEGREES - 360) {
    //         if (Math.abs((targetRotation + 360) - currentPosition) < Math.abs(currentPosition - targetRotation)) {
    //             targetRotation += 360;
    //         }
    //     }

    //     return setTargetFullRotation(targetRotation);
    // }    

    /**
     * Sets the voltage of the turret motor.
     */
    public void setTurretMotorVoltage(double voltage) {
        turretMotor.setVoltage(voltage);
    }

    /**
     * Generates turret velocity from controller inputs.
     * @param fwdPower Forward power from drive controller.
     * @param leftPower Left power from drive controller.
     * @param ccwPower Counter-clockwise rotate power from drive controller.
     * @return The current X and Y velocity of the turret.
     */
    public Transform2d getCurrentVelocity(double fwdPower, double leftPower, double ccwPower) {
        // get robot heading for field transform
        Rotation2d heading = Drive.getPose().getRotation();

        // create transform2d representing current robot-relative chassis velocity
        Transform2d vel = new Transform2d(
            // multiplying -1 to 1 by our max MPS to get current MPS
            fwdPower * SwerveModule.MAX_DRIVE_VEL_MPS, 
            leftPower * SwerveModule.MAX_DRIVE_VEL_MPS, 
            // same here
            Rotation2d.fromDegrees(ccwPower * SwerveModule.MAX_ROTATE_VEL_DPS)
        );

        // Rotate the robot-frame offset into field frame
        Translation2d fieldOffset = turretPosOffset.getTranslation().rotateBy(heading);
    
        // current rotational velocity
        double rotVelRad = vel.getRotation().getRadians();

        // accounting for added turret velocity from chassis rotation (point on spinning rigid body)
        Transform2d robotRelativeVelocity = new Transform2d(
            vel.getX() - rotVelRad * fieldOffset.getY(),
            vel.getY() + rotVelRad * fieldOffset.getX(),
            Rotation2d.kZero
        );

        return robotRelativeVelocity;
    }

    /**
     * Gets the distance from this turret to the hub, accounting for robot velocity.
     * @param chassisPower Current x, y, and rotational duty cycle power being applied to the chassis.
     * @return The adjusted distance, in meters.
     */
    public double getAdjustedHubDistanceMeters(Transform2d chassisPower, double airTime) {
        // grab individual components from transform2d
        double xPower = chassisPower.getX();
        double yPower = chassisPower.getY();
        double rotPower = chassisPower.getRotation().getRadians();

        // get target position on field
        Translation2d targetTranslation2d = 
            AllianceUtil.isRedAlliance() 
            ? FieldConstants.hubRedAlliance.getTranslation() 
            : FieldConstants.hubBlueAlliance.getTranslation();
        
        // get current velocity
        Translation2d currentVelocity = getCurrentVelocity(xPower, yPower, rotPower).getTranslation();

        // get current pose
        Pose2d currPose = Drive.getPose().plus(turretPosOffset);

        // return distance to adjusted target
        return currPose.getTranslation().getDistance(targetTranslation2d.minus(currentVelocity.times(airTime)));
    }

    /**
     * debug
     */
    // public void printEncoderValues() {
    //     SmartDashboard.putNumber("encoder1 " + turretMotor.getDeviceId(), turretCRTEncoder1.getPosition());
    //     SmartDashboard.putNumber("encoder2 " + turretMotor.getDeviceId(), turretCRTEncoder2.get() - secondaryEncoderOffset);
    //     SmartDashboard.putNumber("actualPos " + turretMotor.getDeviceId(), turretEncoder.getPosition());
    // }
}
