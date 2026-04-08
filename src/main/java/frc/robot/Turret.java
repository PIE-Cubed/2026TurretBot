// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AngleKalmanFilter;

/** Add your docs here. */
public class Turret {

    private SparkMax turretMotor;
    private SparkBaseConfig turretMotorConfig;
    
    // private AbsoluteEncoder turretCRTEncoder1;
    // private AbsoluteEncoderConfig turretCRTEncoder1Config;
    // private DutyCycleEncoder turretCRTEncoder2;
    private RelativeEncoder turretEncoder;
    private EncoderConfig turretEncoderConfig;

    private AngleKalmanFilter angleFilter;

    private PIDController turretPID;

    private final double MAX_TURRET_ANGLE_DEGREES = 375;

    private Transform2d turretPosOffset;
    // private TurretCRT crt;

    private int inToleranceCount = 0;

    public Turret(int turretMotorID,// int secondaryEncoderChannel, double secondaryEncoderOffset,
                  Transform2d turretPosOffset, double TURRET_P, double TURRET_I, double TURRET_D, double TURRET_TOLERANCE) {
        this.turretPosOffset = turretPosOffset;

        turretMotor = new SparkMax(turretMotorID, MotorType.kBrushless);
        turretMotorConfig = new SparkMaxConfig()
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(Robot.NEO_550_CURRENT_LIMIT);
        
        turretEncoder = turretMotor.getEncoder();
        turretEncoderConfig = new EncoderConfig().positionConversionFactor(2.8921);

        // turretCRTEncoder1 = turretMotor.getAbsoluteEncoder();
        // turretCRTEncoder1Config = new AbsoluteEncoderConfig().inverted(true);
        // turretMotorConfig.apply(turretCRTEncoder1Config);
        turretMotorConfig.apply(turretEncoderConfig);
        // turretCRTEncoder2 = new DutyCycleEncoder(secondaryEncoderChannel);

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

        angleFilter = new AngleKalmanFilter();
    }

    public int pointAtWithVelocity(Pose2d targetPose, double inAirTime) {
        return pointAt(targetPose.plus(getCurrentVelocity().times(inAirTime)));
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
        double filteredTargetAngle = angleFilter.update(targetRotation);
        Rotation2d targetAbsRotation = new Rotation2d(
            Units.degreesToRadians(filteredTargetAngle - currentRobotRotation)
        );
        // SmartDashboard.putNumber(turretMotor.getDeviceId() + "FieldRelativeTarget", targetRotation);
        // SmartDashboard.putNumber(turretMotor.getDeviceId() + "RobotRelativeTarget", targetAbsRotation.getDegrees());
        return setTargetAbsRotation(targetAbsRotation.getDegrees() + 180);
    }

    // public void printSecondEncoderValue() {
    //     System.out.println(turretCRTEncoder1.getPosition());
    //     System.out.println(turretCRTEncoder2.get());
    //     System.out.println(turretMotor.getDeviceId());
    // }

    public void printCRTResult() {
        // System.out.println(CRT.getCenterGearRotation(turretCRTEncoder1.getPosition(), turretCRTEncoder2.get()));
        System.out.println(turretMotor.getDeviceId());
    }

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

    public Transform2d getCurrentVelocity() {
        return new Transform2d(
            Drive.getVelocity().getX() - Drive.getYawRateRadians() * turretPosOffset.getY(), 
            Drive.getVelocity().getY() + Drive.getYawRateRadians() * turretPosOffset.getX(), 
            Rotation2d.kZero
        );
    }

    public double getAdjustedHubDistance() {
        return Drive.getPose().getTranslation().getDistance((
            AllianceUtil.isRedAlliance() 
            ? FieldConstants.hubRedAlliance.getTranslation() 
            : FieldConstants.hubBlueAlliance.getTranslation()
        ).minus(getCurrentVelocity().getTranslation()));
    }
}
