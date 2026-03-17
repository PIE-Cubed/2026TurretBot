package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * chud shooter class
 */
public class Shooter {

    private SparkFlex leftMotor;
    private SparkBaseConfig leftMotorConfig;

    private SparkFlex rightMotor;
    private SparkBaseConfig rightMotorConfig;

    private SparkMax hoodMotor;
    private SparkBaseConfig hoodMotorConfig;

    private RelativeEncoder leftMotorEncoder;
    private RelativeEncoder rightMotorEncoder;
    private AbsoluteEncoder hoodAbsoluteEncoder;
    private AbsoluteEncoderConfig hoodAbsoluteEncoderConfig;
    private EncoderConfig leftMotorEncoderConfig;
    private EncoderConfig rightMotorEncoderConfig;

    private PIDController leftPIDController;
    private PIDController rightPIDController;
    private PIDController hoodPIDController;

    // Motor IDs
    private final int HOOD_MOTOR_ID = 22;
    private final int LEFT_MOTOR_ID = 21;
    private final int RIGHT_MOTOR_ID = 20;

    // PID Values
    private final double LEFT_P = 0.0003; //The two PIDs need to be different.
    private final double LEFT_I = 0.0;
    private final double LEFT_D = 0.000015;
    private final double LEFT_TOLERANCE = 100.0;

    private final double RIGHT_P = 0.00015;
    private final double RIGHT_I = 0.0;
    private final double RIGHT_D = 0.00006;
    private final double RIGHT_TOLERANCE = 100.0;

    private final double HOOD_P = 1.0;
    private final double HOOD_I = 0.0;
    private final double HOOD_D = 0.0;
    private final double HOOD_TOLERANCE = 0.5;

    private final double HOOD_MIN_ANGLE_DEG = 0;
    private final double HOOD_MAX_ANGLE_DEG = 60;
    private final double HOOD_STOW_ANGLE_DEG = 4;
    private final double FLYWHEEL_STOW_RPM = 2900;

    private double prevRightVoltage = 0;
    private double prevLeftVoltage = 0;

    private int atTargetRPMCount = 0;

    private static final double VELOCITY_TO_VOLT_RATIO = 540;
    private static final double HOOD_ENCODER_CONVERSION = 120;

    private final InterpolatingMatrixTreeMap<Double, N2, N1> distMapMeters = new InterpolatingMatrixTreeMap<>();

    /**
     * chud shooter constructor
     */
    public Shooter() {
        // TODO TODO TODO TODO TODO: tune these again before week 2
        // last recorded on 3/1/2026
        // key = distance to the center of the robot in meters, vector = flywheel RPM and hood angle
        distMapMeters.put(Units.inchesToMeters(0.0), VecBuilder.fill(3375, 9.5)); // not an actual measurement, meant as a crutch for bad odometry
        distMapMeters.put(Units.inchesToMeters(40.5), VecBuilder.fill(3375, 9.5)); //
        distMapMeters.put(Units.inchesToMeters(52.5), VecBuilder.fill(3410, 11.0)); //
        distMapMeters.put(Units.inchesToMeters(64.5), VecBuilder.fill(3450, 13.5)); //
        distMapMeters.put(Units.inchesToMeters(76.5), VecBuilder.fill(3475, 15.0)); //
        distMapMeters.put(Units.inchesToMeters(88.5), VecBuilder.fill(3495, 16.5)); //
        distMapMeters.put(Units.inchesToMeters(100.5), VecBuilder.fill(3520, 17.5)); //
        distMapMeters.put(Units.inchesToMeters(112.5), VecBuilder.fill(3555, 19.0)); //
        distMapMeters.put(Units.inchesToMeters(124.5), VecBuilder.fill(3590, 20.5)); //
        distMapMeters.put(Units.inchesToMeters(136.5), VecBuilder.fill(3615, 22.0)); //
        distMapMeters.put(Units.inchesToMeters(148.5), VecBuilder.fill(3650, 24.5)); //
        distMapMeters.put(Units.inchesToMeters(160.5), VecBuilder.fill(3710, 26.0)); //
        distMapMeters.put(Units.inchesToMeters(172.5), VecBuilder.fill(3910, 27.5)); //
        distMapMeters.put(Units.inchesToMeters(184.5), VecBuilder.fill(4200, 29.5)); //
        distMapMeters.put(Units.inchesToMeters(196.5), VecBuilder.fill(4410, 30.0)); //
        distMapMeters.put(Units.inchesToMeters(208.5), VecBuilder.fill(4610, 31.5)); //

        leftMotor = new SparkFlex(LEFT_MOTOR_ID, MotorType.kBrushless);
        leftMotorConfig = new SparkFlexConfig();

        rightMotor = new SparkFlex(RIGHT_MOTOR_ID, MotorType.kBrushless);
        rightMotorConfig = new SparkFlexConfig();

        hoodMotor = new SparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
        hoodMotorConfig = new SparkMaxConfig();

        leftMotorConfig.idleMode(IdleMode.kCoast);
        leftMotorConfig.smartCurrentLimit(Robot.VORTEX_CURRENT_LIMIT);
        leftMotorConfig.disableFollowerMode();
        leftMotorConfig.inverted(false);

        rightMotorConfig.idleMode(IdleMode.kCoast);
        rightMotorConfig.smartCurrentLimit(Robot.VORTEX_CURRENT_LIMIT);
        rightMotorConfig.disableFollowerMode();
        rightMotorConfig.inverted(true);

        hoodMotorConfig.idleMode(IdleMode.kBrake);
        hoodMotorConfig.smartCurrentLimit(Robot.NEO_CURRENT_LIMIT);
        hoodMotorConfig.inverted(false);

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

        hoodAbsoluteEncoder = hoodMotor.getAbsoluteEncoder();
        hoodAbsoluteEncoderConfig = new AbsoluteEncoderConfig();
        hoodAbsoluteEncoderConfig.positionConversionFactor(HOOD_ENCODER_CONVERSION);
        hoodAbsoluteEncoderConfig.inverted(false);
        hoodMotorConfig.apply(hoodAbsoluteEncoderConfig);

        leftPIDController = new PIDController(LEFT_P, LEFT_I, LEFT_D);
        leftPIDController.setTolerance(LEFT_TOLERANCE);

        rightPIDController = new PIDController(RIGHT_P, RIGHT_I, RIGHT_D);
        rightPIDController.setTolerance(RIGHT_TOLERANCE);
        // hoodFeedForward = new ArmFeedforward(HOOD_FFWD_KS, HOOD_FFWD_KG, HOOD_FFWD_KV, HOOD_FFWD_KA);

        hoodPIDController = new PIDController(HOOD_P, HOOD_I, HOOD_D);
        // hoodPIDController = new ProfiledPIDController(
        //     HOOD_P,
        //     HOOD_I,
        //     HOOD_D,
        //     new TrapezoidProfile.Constraints(HOOD_MAX_VEL_DEG, HOOD_MAX_ACC_DEG)
        // );
        hoodPIDController.setTolerance(HOOD_TOLERANCE);

        leftMotor.configure(leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        hoodMotor.configure(hoodMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setMotorRPM(double rightRPM, double leftRPM) {
        setleftMotorVoltage(rightRPM / VELOCITY_TO_VOLT_RATIO);
        setFlywheelMotorVoltage(leftRPM / VELOCITY_TO_VOLT_RATIO);
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
        hoodMotor.setVoltage(voltage);
    }

    public void setTargetDistance(double distanceMeters, boolean hoodUp) {
        Matrix<N2, N1> mapResult = distMapMeters.get(distanceMeters);
        // SmartDashboard.putNumber("Hub Distance", distanceMeters);

        double targetRPM = mapResult.get(0, 0);
        double targetHoodAngle = mapResult.get(1, 0);
        // SmartDashboard.putNumber("Interpolated RPM", targetRPM);
        // SmartDashboard.putNumber("Interpolated Hood Angle", targetHoodAngle);

        if (hoodUp) {
            setHoodAngle(targetHoodAngle);
            setTargetRPMs(targetRPM, targetRPM);
        } else {
            setHoodAngle(HOOD_STOW_ANGLE_DEG);
            setTargetRPMs(FLYWHEEL_STOW_RPM, FLYWHEEL_STOW_RPM);
        }
    }

    /**
     * Sets voltage of the shooter motors according to target RPMs.
     * @param targetRightRPM Target RPM of the right motor.
     * @param targetleftRPM Target RPM of the left motor.
     */
    public void setTargetRPMs(double targetRightRPM, double targetLeftRPM) {
        double currentRightRPM = rightMotorEncoder.getVelocity();
        double currentLeftRPM = leftMotorEncoder.getVelocity();
        // SmartDashboard.putNumber("Left RPM", currentLeftRPM);
        // SmartDashboard.putNumber("Right RPM", currentRightRPM);
        // SmartDashboard.putNumber("Left Current", leftMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Right Current", rightMotor.getOutputCurrent());

        double rightVoltage = prevRightVoltage;
        rightVoltage = rightVoltage + rightPIDController.calculate(currentRightRPM, targetRightRPM);

        double leftVoltage = prevLeftVoltage;
       
        //System.out.println("Left: " + leftVoltage + ";    PID: " + leftPIDController.calculate(currentLeftRPM, targetLeftRPM));
        leftVoltage = leftVoltage + leftPIDController.calculate(currentLeftRPM, targetLeftRPM);
        
        // SmartDashboard.putNumber("Right Voltage", rightVoltage);
        // SmartDashboard.putNumber("Left Voltage", leftVoltage);

        rightVoltage = MathUtil.clamp(rightVoltage, -12, 12);
        leftVoltage = MathUtil.clamp(leftVoltage, -12, 12);
        //System.out.println("right: " + rightVoltage + "   Left:" + leftVoltage);
        rightMotor.setVoltage(rightVoltage);
        leftMotor.setVoltage(leftVoltage);

        prevRightVoltage = rightVoltage;
        prevLeftVoltage = leftVoltage;
    }

    /**
     * Moves the hood according to a target angle. targetAngleDeg should be between HOOD_MIN_ANGLE_DEG and HOOD_MAX_ANGLE_DEG.
     * @param targetAngleDeg The target angle, in degrees. Clamped to HOOD_MIN_ANGLE_DEG and HOOD_MAX_ANGLE_DEG.
     */
    public void setHoodAngle(double targetAngleDeg) {
        targetAngleDeg = MathUtil.clamp(targetAngleDeg, HOOD_MIN_ANGLE_DEG, HOOD_MAX_ANGLE_DEG);
        // targetAngleDeg = targetAngleDeg + HOOD_OFFSET_DEGREES; 

        double currPositionDeg = hoodAbsoluteEncoder.getPosition();
        // SmartDashboard.putNumber("CurrentHoodAngleDegrees", currPositionDeg);
        // double currVelocityDps = hoodAbsoluteEncoder.getVelocity();

        // double feedback = hoodPIDController.calculate(currPositionDeg, targetAngleDeg);
        // double feedforward = hoodFeedForward.calculateWithVelocities(
        //     Math.toRadians(currPositionDeg),
        //     Math.toRadians(currVelocityDps),
        //     Math.toRadians(hoodPIDController.getSetpoint().velocity)
        // );

        // double voltage = feedback + feedforward;

        double voltage = hoodPIDController.calculate(currPositionDeg, targetAngleDeg);

        hoodMotor.setVoltage(voltage);
    }

    public void printWheelRPMs() {
        double currentFlywheelRPM = rightMotorEncoder.getVelocity();
        double currentLeftRPM = leftMotorEncoder.getVelocity();

        SmartDashboard.putNumber("Shooter/CurrentFlywheelRPM", currentFlywheelRPM);
        SmartDashboard.putNumber("Shooter/CurrentLeftRPM", currentLeftRPM);
    }

    public void stopWheels() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();

        prevLeftVoltage = 0;
        prevRightVoltage = 0;
    }

    public void stopHood() {
        hoodMotor.stopMotor();
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
/* FOR MOTOR SET FUCNTION
 * .1 power = 606 RPM
 * .2 power = 1240 RPM
 * .3 power = 1866 RPM
 * .4 power = 2500 RPM
 * .5 power = 3140 RPM
 * .6 power = 3783 RPM
 * .7 power = 4408 RPM
 * .8 power = 5012 RPM
 * .9 power = 5600 RPM
 *  1 power = 6175 RPM
 *
 * MOTORSETVOLTAGE (More Stable than SET FUNCTOIN)
 * volts  RPM    Delta   RPM/Volts
 * 1      527            527
 * 2      1085   558     542.5
 * 3      1642   560     547.3
 * 4      2201   559     550.25
 * 5      2761   560     552.2
 * 6      3322   561     553.67
 * 7      3884   562     554.85
 * 8      4446   562     555.75
 * 9      5006   560     556.22
 * 10     5550   544     555
 * 11     6090   540     553.64
 * MAX 12 6155   065     512.92
 */






 /* Distance (Inches) from center of the hub to the front of the bumper
  * Distance    RPM (Both Shooters)    Hood Angle
  * 24             3755                   5.5
  * 36             3800                   7.5
  * 48             3845                   9.5
  * 60             3890                  11.5
  * 72             3925                  12.5
  * 84             3970                  14.5
  * 96             4015 `                16.5
  * 108            4005                  18.5
  * 120            3990                  20.5
  * 132            3975                  22.5
  * 144            4015                  24.5
  */

// 02/28/2026
  /* Distance (Inches) from center of the hub to the front of the bumper
  * Distance    RPM (Both Shooters)    Hood Angle
  * 24             3750                   6.5
  * 36             3790                   7.5
  * 48             3790                  10.0
  * 60             3790                  11.5
  * 72             3790                  13.0
  * 84             3790                  14.5
  * 96             3830 `                17.0
  * 108            3850                  19.0
  * 120            3880                  21.5
  * 132            3920                  24.0
  * 144            4020                  26.0
  */
