package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Drive.PositionState;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Logger;

/**
 * chud shooter class
 */
public class Shooter {

    private Turret leftTurret;
    private final Transform2d leftTurretOffset = new Transform2d(
        Units.inchesToMeters(-4.75), Units.inchesToMeters(6.562500), Rotation2d.kZero);
    private Turret rightTurret;
    private final Transform2d rightTurretOffset = new Transform2d(
        Units.inchesToMeters(-4.75), Units.inchesToMeters(-6.562500), Rotation2d.kZero);

    private SparkFlex leftMotor;
    private SparkBaseConfig leftMotorConfig;

    private SparkFlex rightMotor;
    private SparkBaseConfig rightMotorConfig;

    private SparkMax leftHoodMotor;
    private SparkBaseConfig leftHoodMotorConfig;
    
    private SparkMax rightHoodMotor;
    private SparkBaseConfig rightHoodMotorConfig;

    private RelativeEncoder leftMotorEncoder;
    private EncoderConfig leftMotorEncoderConfig;
    private RelativeEncoder rightMotorEncoder;
    private EncoderConfig rightMotorEncoderConfig;
    private RelativeEncoder leftHoodEncoder;
    private EncoderConfig leftHoodEncoderConfig;
    private RelativeEncoder rightHoodEncoder;
    private EncoderConfig rightHoodEncoderConfig;

    private PIDController leftPIDController;
    private PIDController rightPIDController;
    private PIDController leftHoodPIDController;
    private PIDController rightHoodPIDController;

    // Motor IDs
    private final int LEFT_TURRET_MOTOR_ID = 45;
    private final int RIGHT_TURRET_MOTOR_ID = 44;
    private final int LEFT_HOOD_MOTOR_ID = 43;
    private final int RIGHT_HOOD_MOTOR_ID = 42;
    private final int LEFT_MOTOR_ID = 41;
    private final int RIGHT_MOTOR_ID = 40;

    private final int LEFT_TURRET_ENCODER_CHANNEL = 0;
    private final int RIGHT_TURRET_ENCODER_CHANNEL = 1;

    private final double LEFT_TURRET_ENCODER_OFFSET = 0.9672; // 0 to 1
    private final double RIGHT_TURRET_ENCODER_OFFSET = 0.5193; // 0 to 1

    // PID Values
    private final double LEFT_F = 0.001968;
    private final double LEFT_P = 0.00201;
    private final double LEFT_I = 0.0;
    private final double LEFT_D = 0.00007;
    private final double LEFT_TOLERANCE = 100.0;

    private final double RIGHT_F = 0.002;
    private final double RIGHT_P = 0.0028;
    private final double RIGHT_I = 0.0;
    private final double RIGHT_D = 0.00006;
    private final double RIGHT_TOLERANCE = 100.0;

    private final double LEFT_HOOD_P = 0.5;
    private final double LEFT_HOOD_I = 0.0;
    private final double LEFT_HOOD_D = 0.01;
    private final double LEFT_HOOD_S = 0.06;
    private final double RIGHT_HOOD_P = 0.5;
    private final double RIGHT_HOOD_I = 0.0;
    private final double RIGHT_HOOD_D = 0.01;
    private final double RIGHT_HOOD_S = 0.06;
    private final double HOOD_TOLERANCE = 0.25;

    private final double HOOD_MIN_ANGLE_DEG = 0;
    private final double HOOD_MAX_ANGLE_DEG = 20;
    private final double HOOD_STOW_ANGLE_DEG = 0;

    private final double LEFT_TURRET_P = 0.55;
    private final double LEFT_TURRET_I = 0;
    private final double LEFT_TURRET_D = 0.01;
    private final double LEFT_TURRET_TOLERANCE = 0.5;

    private final double RIGHT_TURRET_P = 0.55;
    private final double RIGHT_TURRET_I = 0;
    private final double RIGHT_TURRET_D = 0.01;
    private final double RIGHT_TURRET_TOLERANCE = 0.5;

    private double leftDistAdjust = 0;
    private double rightDistAdjust = 0;

    private int atTargetRPMCount = 0;

    private static final double HOOD_ENCODER_CONVERSION = 20.0 / 214.0 * 360.0 / 12.0;

    private final InterpolatingMatrixTreeMap<Double, N4, N1> distMapMeters = new InterpolatingMatrixTreeMap<>();

    /**
     * I love my chud son Terry the Turret bot :D
     */
    public Shooter() {
        // last recorded on 4/29/2026
        // key = distance to the center of the robot in meters, vector = RPM (left), angle (left), RPM (right), angle (right)
        distMapMeters.put(Units.inchesToMeters(0.0), VecBuilder.fill(2640, 6.25, 2640, 6.25)); 
        //                   ^^ not an actual measurement, meant as a crutch for bad odometry ^^

        distMapMeters.put(Units.inchesToMeters(0.0 + 45.25 /* 45.25 */),  VecBuilder.fill(  2640, 6.25,    2640, 6.25));
        distMapMeters.put(Units.inchesToMeters(12.0 + 45.25 /* 57.25 */),  VecBuilder.fill( 2650, 8.25,    2650, 8.25));
        distMapMeters.put(Units.inchesToMeters(24.0 + 45.25 /* 69.25 */),  VecBuilder.fill( 2665, 10.25,   2665, 10.25));
        distMapMeters.put(Units.inchesToMeters(36.0 + 45.25 /* 81.25 */),  VecBuilder.fill( 2680, 12.0,    2680, 12.0));
        distMapMeters.put(Units.inchesToMeters(48.0 + 45.25 /* 93.25 */),  VecBuilder.fill( 2705, 13.5,    2705, 13.5));
        distMapMeters.put(Units.inchesToMeters(60.0 + 45.25 /* 105.25 */), VecBuilder.fill( 2745, 15.0,    2745, 15.0));
        distMapMeters.put(Units.inchesToMeters(72.0 + 45.25 /* 117.25 */), VecBuilder.fill( 2840, 16.5,    2840, 16.5));
        distMapMeters.put(Units.inchesToMeters(84.0 + 45.25 /* 129.25 */), VecBuilder.fill( 2960, 18.0,    2960, 18.0));
        distMapMeters.put(Units.inchesToMeters(96.0 + 45.25 /* 141.25 */), VecBuilder.fill( 3090, 20.25,   3090, 20.25));
        distMapMeters.put(Units.inchesToMeters(108.0 + 45.25 /* 153.25 */), VecBuilder.fill(3235, 22.5,    3235, 22.5));
        distMapMeters.put(Units.inchesToMeters(120.0 + 45.25 /* 165.25 */), VecBuilder.fill(3403, 24.86,   3403, 24.86)); // extrapolated
        distMapMeters.put(Units.inchesToMeters(132.0 + 45.25 /* 177.25 */), VecBuilder.fill(3601, 27.34,   3601, 27.34)); // extrapolated
        distMapMeters.put(Units.inchesToMeters(144.0 + 45.25 /* 189.25 */), VecBuilder.fill(3835, 29.94,   3835, 29.94)); // extrapolated
        distMapMeters.put(Units.inchesToMeters(156.0 + 45.25 /* 201.25 */), VecBuilder.fill(4109, 32.66,   4109, 32.66)); // extrapolated
        distMapMeters.put(Units.inchesToMeters(168.0 + 45.25 /* 213.25 */), VecBuilder.fill(4428, 35.50,   4428, 35.50)); // extrapolated
        distMapMeters.put(Units.inchesToMeters(-1.0 /* short pass */), VecBuilder.fill(3300, 23.0, 3300, 23.0));
        distMapMeters.put(Units.inchesToMeters(-2.0 /* long pass */), VecBuilder.fill(4500, 24.0, 4500, 24.0)); // guess

        leftTurret = new Turret(
            LEFT_TURRET_MOTOR_ID, LEFT_TURRET_ENCODER_CHANNEL, LEFT_TURRET_ENCODER_OFFSET,
            leftTurretOffset, LEFT_TURRET_P, LEFT_TURRET_I, LEFT_TURRET_D, LEFT_TURRET_TOLERANCE
        );

        rightTurret = new Turret(
            RIGHT_TURRET_MOTOR_ID, RIGHT_TURRET_ENCODER_CHANNEL, RIGHT_TURRET_ENCODER_OFFSET,
            rightTurretOffset, RIGHT_TURRET_P, RIGHT_TURRET_I, RIGHT_TURRET_D, RIGHT_TURRET_TOLERANCE
        );

        leftMotor = new SparkFlex(LEFT_MOTOR_ID, MotorType.kBrushless);
        leftMotorConfig = new SparkFlexConfig();

        rightMotor = new SparkFlex(RIGHT_MOTOR_ID, MotorType.kBrushless);
        rightMotorConfig = new SparkFlexConfig();

        leftHoodMotor = new SparkMax(LEFT_HOOD_MOTOR_ID, MotorType.kBrushless);
        leftHoodMotorConfig = new SparkMaxConfig();

        rightHoodMotor = new SparkMax(RIGHT_HOOD_MOTOR_ID, MotorType.kBrushless);
        rightHoodMotorConfig = new SparkMaxConfig();

        leftMotorConfig.idleMode(IdleMode.kCoast);
        leftMotorConfig.smartCurrentLimit(80);
        leftMotorConfig.disableFollowerMode();
        leftMotorConfig.inverted(false);
        leftMotorConfig.secondaryCurrentLimit(90);

        rightMotorConfig.idleMode(IdleMode.kCoast);
        rightMotorConfig.smartCurrentLimit(80);
        rightMotorConfig.disableFollowerMode();
        rightMotorConfig.inverted(false);
        rightMotorConfig.secondaryCurrentLimit(90);

        leftHoodMotorConfig.idleMode(IdleMode.kBrake);
        leftHoodMotorConfig.smartCurrentLimit(Robot.NEO_550_CURRENT_LIMIT);
        leftHoodMotorConfig.inverted(true);
        leftHoodMotorConfig.disableFollowerMode();

        rightHoodMotorConfig.idleMode(IdleMode.kBrake);
        rightHoodMotorConfig.smartCurrentLimit(Robot.NEO_550_CURRENT_LIMIT);
        rightHoodMotorConfig.inverted(true);
        rightHoodMotorConfig.disableFollowerMode();

        leftMotorEncoder = leftMotor.getEncoder();
        leftMotorEncoderConfig = new EncoderConfig();
        leftMotorEncoderConfig.positionConversionFactor(1.0);
        leftMotorEncoderConfig.velocityConversionFactor(1.0);
        leftMotorConfig.apply(leftMotorEncoderConfig);

        rightMotorEncoder = rightMotor.getEncoder();
        rightMotorEncoderConfig = new EncoderConfig();
        rightMotorEncoderConfig.positionConversionFactor(1.0);
        rightMotorEncoderConfig.velocityConversionFactor(1.0);
        rightMotorConfig.apply(rightMotorEncoderConfig);

        leftHoodEncoder = leftHoodMotor.getEncoder();
        leftHoodEncoderConfig = new EncoderConfig();
        leftHoodEncoderConfig.positionConversionFactor(HOOD_ENCODER_CONVERSION);
        leftHoodMotorConfig.apply(leftHoodEncoderConfig);
        leftHoodEncoder.setPosition(0);

        rightHoodEncoder = rightHoodMotor.getEncoder();
        rightHoodEncoderConfig = new EncoderConfig();
        rightHoodEncoderConfig.positionConversionFactor(HOOD_ENCODER_CONVERSION);
        rightHoodMotorConfig.apply(rightHoodEncoderConfig);
        rightHoodEncoder.setPosition(0);

        leftPIDController = new PIDController(LEFT_P, LEFT_I, LEFT_D);
        leftPIDController.setTolerance(LEFT_TOLERANCE);

        rightPIDController = new PIDController(RIGHT_P, RIGHT_I, RIGHT_D);
        rightPIDController.setTolerance(RIGHT_TOLERANCE);

        leftHoodPIDController = new PIDController(LEFT_HOOD_P, LEFT_HOOD_I, LEFT_HOOD_D);
        leftHoodPIDController.setTolerance(HOOD_TOLERANCE);

        rightHoodPIDController = new PIDController(RIGHT_HOOD_P, RIGHT_HOOD_I, RIGHT_HOOD_D);
        rightHoodPIDController.setTolerance(HOOD_TOLERANCE);

        leftMotor.configure(leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        leftHoodMotor.configure(leftHoodMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightHoodMotor.configure(rightHoodMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void turretAdjust(double leftTurretAdjust, double rightTurretAdjust) {
        leftTurret.nudgeEncoder(leftTurretAdjust);
        rightTurret.nudgeEncoder(rightTurretAdjust);
    }

    public void distAdjust(double leftDistAdjust, double rightDistAdjust) {
        this.leftDistAdjust += this.leftDistAdjust + leftDistAdjust;
        this.rightDistAdjust += this.rightDistAdjust + rightDistAdjust;
    }

    public void nudgeAim(Translation2d leftAdjust, Translation2d rightAdjust) {
        turretAdjust(leftAdjust.getX(), rightAdjust.getX());
        distAdjust(leftAdjust.getY(), rightAdjust.getY());

        Logger.logStruct("leftAdjust", leftAdjust);
        Logger.logStruct("rightAdjust", rightAdjust);
    }

    /**
     * voltage should be -12 to 12
     * @param rightVoltage
     * @param leftVoltage
     */
    public void setWheelVoltages(double rightVoltage, double leftVoltage) {
        setFlywheelMotorVoltage(rightVoltage);
        setleftMotorVoltage(leftVoltage);
    }

    /**
     * voltage should be -12 to 12
     * @param voltage
     */
    public void setleftMotorVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    /**
     * voltage should be -12 to 12
     * @param voltage
     */
    public void setFlywheelMotorVoltage(double voltage) {
        rightMotor.setVoltage(voltage);
    }

    /**
     * voltage should be -12 to 12
     * @param voltage
     */
    public void setHoodMotorVoltage(double voltage) {
        leftHoodMotor.setVoltage(voltage);
    }

    public void stopTurrets() {
        leftTurret.setTurretMotorVoltage(0);
        rightTurret.setTurretMotorVoltage(0);
    }

    public void autoAdjust(boolean hoodUp) {
        autoAdjust(hoodUp, Transform2d.kZero, Translation2d.kZero, true, false);
    }

    public void autoAdjust(boolean hoodUp, Transform2d robotVel, Translation2d aimAdjust, boolean fieldDrive, boolean pass) {
        Pose2d targetPose = null;
        double y = Drive.getPose().getY();
        PositionState currPositionState = Drive.getPositionState();

        double leftDist = leftTurret.getAdjustedHubDistance(robotVel) + leftDistAdjust;
        double rightDist = rightTurret.getAdjustedHubDistance(robotVel) + rightDistAdjust;

        if (pass) {
            if (currPositionState == PositionState.AWAY) {
                leftDist = -2;
                rightDist = -2;
                if (y > 4.04) {
                    if (AllianceUtil.isRedAlliance()) {
                        targetPose = new Pose2d(11.8, 7.3, Rotation2d.kZero);
                    } else {
                        targetPose = new Pose2d(4.8, 7.3, Rotation2d.kZero);
                    }
                } else {
                    if (AllianceUtil.isRedAlliance()) {
                        targetPose = new Pose2d(11.8, 0.75, Rotation2d.kZero);
                    } else {
                        targetPose = new Pose2d(4.8, 0.75, Rotation2d.kZero);
                    }
                }
            } else if (currPositionState == PositionState.MID) {
                leftDist = -1;
                rightDist = -1;
                if (y > 4.04) {
                    if (AllianceUtil.isRedAlliance()) {
                        targetPose = new Pose2d(16.0, 6.2, Rotation2d.kZero);
                    } else {
                        targetPose = new Pose2d(1.0, 6.2, Rotation2d.kZero);
                    }
                } else {
                    if (AllianceUtil.isRedAlliance()) {
                        targetPose = new Pose2d(16.0, 1.9, Rotation2d.kZero);
                    } else {
                        targetPose = new Pose2d(1.0, 1.9, Rotation2d.kZero);
                    }
                }
            } else {
                targetPose = 
                    ((AllianceUtil.isRedAlliance()) ? FieldConstants.hubRedAlliance : FieldConstants.hubBlueAlliance);
            }
        } else {
            targetPose = 
                ((AllianceUtil.isRedAlliance()) ? FieldConstants.hubRedAlliance : FieldConstants.hubBlueAlliance);
        }
        
        Matrix<N4, N1> mapResult = distMapMeters.get(leftDist);
        
        double targetLeftRPM = mapResult.get(0, 0);
        double targetLeftHoodAngle = mapResult.get(1, 0);
        double ballAirTimeLeft = getFlightTime(leftDist);

        mapResult = distMapMeters.get(rightDist);
        
        double targetRightRPM = mapResult.get(2, 0);
        double targetRightHoodAngle = mapResult.get(3, 0);
        double ballAirTimeRight = getFlightTime(rightDist);

        double targetTheta = Math.atan2(targetPose.getY() - Drive.getPose().getY(), targetPose.getX() - Drive.getPose().getX());

        targetPose = targetPose.plus(new Transform2d(aimAdjust.rotateBy(Rotation2d.fromRadians(targetTheta)), Rotation2d.kZero));

        if (!fieldDrive) {
            robotVel = new Transform2d(robotVel.getTranslation().rotateBy(Drive.getPose().getRotation()), robotVel.getRotation());
        }

        leftTurret.pointAtWithVelocity(targetPose, ballAirTimeLeft, robotVel);
        rightTurret.pointAtWithVelocity(targetPose, ballAirTimeRight, robotVel);
        setTargetRPMs(targetRightRPM, targetLeftRPM);

        if (hoodUp) {
            setHoodAngle(targetLeftHoodAngle, targetRightHoodAngle);
        } else {
            setHoodAngle(HOOD_STOW_ANGLE_DEG, HOOD_STOW_ANGLE_DEG);
        }

        leftTurret.printEncoderValues();
        rightTurret.printEncoderValues();
    }

    private double getFlightTime(double distanceMeters) {
        if (distanceMeters < 0) {
            return 0;
        }

        final double G        = 32.174;
        final double V0       = 23.5; // measured at 3000 rpm
        final double H_LAUNCH = 16.0 / 12.0;
        final double H_TARGET = 56.5 / 12.0;

        // equation needs angle from flat, but we store angle from vertical.
        double rad = Math.toRadians(90 - distMapMeters.get(distanceMeters).get(1, 0));
        double rpm = distMapMeters.get(distanceMeters).get(0, 0);

        // adjusting the measured speed for our current rpm
        double Vy  = (rpm / 3000) * V0 * Math.sin(rad);

        double a = -0.5 * G;
        double b = Vy;
        double c = H_LAUNCH - H_TARGET;

        // b^2 - 4ac
        double discriminant = (b * b) - (4 * a * c);
        if (discriminant < 0) return 0.8;

        // (-b +/- discriminant) / 2a
        double t1 = (-b + Math.sqrt(discriminant)) / (2 * a);
        double t2 = (-b - Math.sqrt(discriminant)) / (2 * a);

        if (t1 > 0 && t2 > 0) return Math.max(t1, t2);
        if (t1 > 0) return t1;
        if (t2 > 0) return t2;
        return 0.8;
    }

    /**
     * Sets voltage of the shooter motors according to target RPMs.
     * @param targetRightRPM Target RPM of the right motor.
     * @param targetleftRPM Target RPM of the left motor.
     */
    public void setTargetRPMs(double targetRightRPM, double targetLeftRPM) {
        double currentRightRPM = rightMotorEncoder.getVelocity();
        double currentLeftRPM = leftMotorEncoder.getVelocity();

        SmartDashboard.putNumber("Left RPM", currentLeftRPM);
        SmartDashboard.putNumber("Right RPM", currentRightRPM);

        double leftVoltage = LEFT_F * targetLeftRPM + leftPIDController.calculate(currentLeftRPM, targetLeftRPM);
        double rightVoltage = RIGHT_F * targetRightRPM + rightPIDController.calculate(currentRightRPM, targetRightRPM);

        rightVoltage = MathUtil.clamp(rightVoltage, -12, 12);
        leftVoltage = MathUtil.clamp(leftVoltage, -12, 12);

        rightMotor.setVoltage(rightVoltage);
        leftMotor.setVoltage(leftVoltage);
    }

    /**
     * Moves the hood according to a target angle. targetAngleDeg should be between HOOD_MIN_ANGLE_DEG and HOOD_MAX_ANGLE_DEG.
     * @param targetLeftAngleDeg The target angle, in degrees. Clamped to HOOD_MIN_ANGLE_DEG and HOOD_MAX_ANGLE_DEG.
     */
    public void setHoodAngle(double targetLeftAngleDeg, double targetRightAngleDeg) {
        targetLeftAngleDeg = MathUtil.clamp(targetLeftAngleDeg, HOOD_MIN_ANGLE_DEG, HOOD_MAX_ANGLE_DEG);
        targetRightAngleDeg = MathUtil.clamp(targetRightAngleDeg, HOOD_MIN_ANGLE_DEG, HOOD_MAX_ANGLE_DEG);

        SmartDashboard.putNumber("Target Left Hood", targetLeftAngleDeg);
        SmartDashboard.putNumber("Target Right Hood", targetRightAngleDeg);

        double currLeftPositionDeg = leftHoodEncoder.getPosition();
        double currRightPositionDeg = rightHoodEncoder.getPosition();

        double leftVoltage = leftHoodPIDController.calculate(currLeftPositionDeg, targetLeftAngleDeg);
        leftVoltage = MathUtil.clamp(leftVoltage + LEFT_HOOD_S, -12, 12);
        double rightVoltage = rightHoodPIDController.calculate(currRightPositionDeg, targetRightAngleDeg);
        rightVoltage = MathUtil.clamp(rightVoltage + RIGHT_HOOD_S, -12, 12);

        leftHoodMotor.setVoltage(leftVoltage);
        rightHoodMotor.setVoltage(rightVoltage);
    }

    public void printWheelRPMs() {
        SmartDashboard.putNumber("Shooter/CurrentRightRPM", rightMotorEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter/CurrentLeftRPM", leftMotorEncoder.getVelocity());
    }

    public void stopWheels() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    public void testFunction() {
        
    }

    public void stopHood() {
        leftHoodMotor.stopMotor();
        rightHoodMotor.stopMotor();
    }

    public boolean atTargetRPM() {
        if (leftPIDController.atSetpoint() && rightPIDController.atSetpoint()) {
            atTargetRPMCount ++;
        } else {
            atTargetRPMCount = 0;
        }

        return atTargetRPMCount >= 10;
    }

    /******************************************************************************************************
     *
     * TEST PROGRAMS
     *
     ******************************************************************************************************/

    public void testVoltageVsVelocity(double voltage) {
        double velocity;

        rightMotor.setVoltage(voltage);
        velocity = rightMotorEncoder.getVelocity();
        System.out.println("velocity = " + velocity + "    " + voltage);
    }
}