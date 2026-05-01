// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drive.PositionState;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Elastic;
import frc.robot.util.Logger;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

/**
 * Remember to feed the robot at least 3 FIRST brand lemons each day to keep it happy and bug-free.
 */
public class Robot extends TimedRobot {
    public static double totalCurrent = 0;

    public static final int FAIL = -1;
    public static final int PASS = 1;
    public static final int DONE = 2;
    public static final int CONT = 3;

    public static final int NEO_CURRENT_LIMIT = 60;
    public static final int NEO_550_CURRENT_LIMIT = 30;
    public static final int VORTEX_CURRENT_LIMIT = 80;

    private static final String kNoAuto = "No Auto";
    private static final String kTestAuto = "Test Auto";
    private static final String kCenterOutAuto = "Center Outpost Auto";
    private static final String kCenterDepAuto = "Center Depot Auto";
    private static final String kOutpostAuto = "Outpost Auto (Safe)";
    private static final String kRiskyOutpostAuto = "Outpost Auto (Risky)";
    private static final String kDepotAuto = "Depot Auto (Safe)";
    private static final String kRiskyDepotAuto = "Depot Auto (Risky)";
    private static final String kMod1 = "Climb";
    private static final String kMod2 = "No Climb";
    private String m_sideSelected;
    private String m_modSelected;
    private final SendableChooser<String> side_chooser = new SendableChooser<>();
    private final SendableChooser<String> mod_chooser = new SendableChooser<>();

    private enum WheelState {
        TELEOP,
        LOCK_WHEELS,
        AUTO_DRIVE,
        AUTO_AIM,
    }

    WheelState currentWheelState = WheelState.TELEOP;

    double currentAdjustedHubDistance = 0;

    // private PowerDistribution pdh;

    private final Field2d field2d = new Field2d();

    private Controls controls;
    private Drive drive;
    private Shooter shooter;
    private Hopper hopper;
    private Grabber grabber;
    private Odometry odometry;
    private Auto auto;

    private Timer dsConnectTimer = new Timer();

    private Optional<Trajectory<SwerveSample>> testAuto;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        side_chooser.setDefaultOption("No Auto", kNoAuto);
        // side_chooser.addOption("Outpost (Risky)", kRiskyOutpostAuto);
        side_chooser.addOption("Outpost", kOutpostAuto);
        // side_chooser.addOption("Center", kCenterOutAuto);
        side_chooser.addOption("Center -> Outpost", kCenterOutAuto);
        side_chooser.addOption("Center -> Depot", kCenterDepAuto);
        // side_chooser.addOption("Depot (Risky)", kRiskyDepotAuto);
        side_chooser.addOption("Depot", kDepotAuto);
        // side_chooser.addOption("Test", kTestAuto);

        mod_chooser.setDefaultOption("Mod 1", kMod1);
        mod_chooser.addOption("Mod 2", kMod2);

        SmartDashboard.putData("Auto | Side Chooser", side_chooser);
        SmartDashboard.putData("Auto | Mod Chooser", mod_chooser);

        SmartDashboard.putNumber("TargetLeftHoodAngleDegrees", 0);
        SmartDashboard.putNumber("TargetLeftWheelRPM", 0);
        SmartDashboard.putNumber("TargetRightHoodAngleDegrees", 0);
        SmartDashboard.putNumber("TargetRightWheelRPM", 0);

        // SmartDashboard.putNumber("ShooterRPM", 0);
        // SmartDashboard.putNumber("HoodAngle", 0);

        // pdh = new PowerDistribution(2, ModuleType.kRev);

        controls = new Controls();
        drive = new Drive();
        shooter = new Shooter();
        hopper = new Hopper();
        grabber = new Grabber();
        odometry = new Odometry(drive);
        auto = new Auto(drive, shooter, hopper, grabber);

        // drive.resetPose(new Pose2d(Drive.SWERVE_DIST_FROM_CENTER, Drive.SWERVE_DIST_FROM_CENTER, Rotation2d.kZero));

        dsConnectTimer.restart();
        DriverStation.waitForDsConnection(0);
        dsConnectTimer.stop();

        // Elastic.selectTab("Setup");
        Elastic.sendNotification(
            new Notification(
                NotificationLevel.INFO, 
                "DS Connected in " + dsConnectTimer.get() + " seconds.", 
                DriverStation.getMatchType().equals(MatchType.Elimination) ?
                "high cortisol drive team" : "low cortisol drive team"
            )
            .withAutomaticHeight()
            .withDisplaySeconds(5)
        );

        // SmartDashboard.putNumber("CurrPosX", 0);
        // SmartDashboard.putNumber("CurrPosY", 0);
        // SmartDashboard.putNumber("CurrPosT", 0);

        testAuto = Choreo.loadTrajectory("testAuto");
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // SmartDashboard.putNumber("Voltage", pdh.getVoltage());

        // Odometry and Pose
        odometry.updateVisionEstimators();

        drive.updatePoseEstimator();

        if (odometry.getCamera1Pose() != null) {
            drive.addVisionMeasurement(odometry.getCamera1Pose(), Odometry.CAMERA1_STD_DEVS);
            Logger.logStruct("BRSwerveCameraPose", odometry.getCamera1Pose().estimatedPose);
        }

        if (odometry.getCamera2Pose() != null) {
            drive.addVisionMeasurement(odometry.getCamera2Pose(), Odometry.CAMERA2_STD_DEVS);
            Logger.logStruct("BLSwerveCameraPose", odometry.getCamera2Pose().estimatedPose);
        }

        if (odometry.getCamera3Pose() != null) {
            drive.addVisionMeasurement(odometry.getCamera3Pose(), Odometry.CAMERA3_STD_DEVS);
            Logger.logStruct("FRBarCameraPose", odometry.getCamera3Pose().estimatedPose);
        }

        if (odometry.getCamera4Pose() != null) {
            drive.addVisionMeasurement(odometry.getCamera4Pose(), Odometry.CAMERA4_STD_DEVS);
            Logger.logStruct("FLBarCameraPose", odometry.getCamera4Pose().estimatedPose);
        }

        Logger.logStruct("currentPose2d", Drive.getPose());

        field2d.setRobotPose(Drive.getPose());
        SmartDashboard.putData("Field", field2d);

        // // Match Time and Hub State
        double matchTime = DriverStation.getMatchTime();
        SmartDashboard.putNumber("Match Time", matchTime);
        SmartDashboard.putNumber("Time Until Shift", AllianceUtil.timeUntilHubStateChange(matchTime));
        SmartDashboard.putBoolean("Hub Active", AllianceUtil.isOurHubActive(matchTime));

        shooter.printWheelRPMs();

        m_sideSelected = side_chooser.getSelected();
        m_modSelected = mod_chooser.getSelected();

        drive.printSwerveState();

        totalCurrent = 0;

        shooter.log();
        hopper.log();
        grabber.log();
        drive.log();

        SmartDashboard.putNumber("currents/totalCurrent", totalCurrent);
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        m_sideSelected = side_chooser.getSelected();
        m_modSelected = mod_chooser.getSelected();

        Elastic.selectTab("Autonomous");
        Elastic.sendNotification(
            new Notification(
                NotificationLevel.INFO, 
                "Autonomous Start", 
                "Drivers behind the lines..."
            )
            .withAutomaticHeight()
            .withDisplaySeconds(3)
        );

        shooter.zeroTurrets();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        boolean modifier = m_modSelected == kMod2;

        switch (m_sideSelected) {
            case kCenterDepAuto:
                auto.centerDepAuto();
                break;
            case kCenterOutAuto:
                auto.centerOutAuto();
                break;
            case kOutpostAuto:
                auto.outpostAuto(modifier);
                break;
            case kDepotAuto:
                auto.depotAuto(modifier);
                break;
            case kNoAuto:
                break;
            default:
                // Put default auto code here
                break;
        }
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        Elastic.selectTab("Teleop");
        Elastic.sendNotification(
            new Notification(
                NotificationLevel.INFO, 
                "Teleop Start", 
                "GO! GO! GO!"
            )
            .withAutomaticHeight()
            .withDisplaySeconds(3)
        );
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        wheelControl();
        // testShooterControl();
        shooterControl();
        grabberControl();
        elasticControl();
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // climber.zeroClimberEncoder();
        auto.restartTimer();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        // wheelControl();
        // testShooterControl();
        // SmartDashboard.putNumber("Hub Distance", Units.metersToInches(drive.getHubDistanceMeters()));
        // grabberControl();
        //hopper.indexFuel();
        //shooterControl();
        //shooter.setTargetRPMs(3800,3800);
        // shooter.setHoodAngle(SmartDashboard.getNumber("TargetHoodAngleDegrees", 0));
        // climberControl();
        // grabberControl();

        // auto.choreoPathFollower(testAuto);

        // testShooterControl();

        // shooter.stopTurrets();
        // // shooter.testFunction();
        shooter.setHoodAngle(20, 20);
        // shooter.setTargetRPMs(2500, 2500);

        // boolean shootButton = controls.getShootButton();

        // if (shootButton) {
        //     hopper.indexFuel();
        // } else {
        //     hopper.stopMotors();
        // }

        // climber.zeroClimberEncoder();
        // manualClimberControl();

        // shooter.stopHood();
        // shooter.stopWheels();
        // grabber.stopGrabber();
        // grabber.stopWheel();
        // drive.stopWheels();
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}

    private void wheelControl() {
        boolean resetGyro = controls.resetGyro();
        boolean fieldDrive = controls.getFieldDrive();
        boolean lockWheels = controls.getWheelLock();
        // boolean autoAim = controls.getAutoAim();

        double forwardPowerFwdPos = controls.getForwardPowerFwdPositive();
        double strafePowerLeftPos = controls.getStrafePowerLeftPositive();
        double rotatePowerCcwPos = controls.getRotatePowerCcwPositive();
        double rightStickY = controls.getRightY();

        currentAdjustedHubDistance = drive.getAdjustedHubDistanceMeters(forwardPowerFwdPos, strafePowerLeftPos, fieldDrive);

        if (resetGyro) {
            drive.resetGyro();
        }

        if (lockWheels) {
            currentWheelState = WheelState.LOCK_WHEELS;
        // } else if (autoAim) {
        //     currentWheelState = WheelState.AUTO_AIM;
        } else {
            currentWheelState = WheelState.TELEOP;
        }

        if (currentWheelState == WheelState.LOCK_WHEELS) {
            drive.lockWheels();
        // } else if (currentWheelState == WheelState.AUTO_AIM) {
        //     drive.shootAndDrive(forwardPowerFwdPos, strafePowerLeftPos, fieldDrive);
        } else if (currentWheelState == WheelState.TELEOP) {
            drive.teleopDrive(
                forwardPowerFwdPos,
                strafePowerLeftPos,
                rotatePowerCcwPos,
                fieldDrive,
                drive.getCenterOfRotation(rotatePowerCcwPos, rightStickY)
            );
        }
    }

    public void shooterControl() {
        boolean shootButton = controls.getShootButton();
        boolean reverseIndexer = controls.getReverseIndexer();

        PositionState currentPositionState = Drive.getPositionState();

        boolean shootReady = true;

        if (currentPositionState == PositionState.TRENCH) {
            shootReady = false;
        }

        // we always run the flywheel
        // System.out.println("current distance to hub: " + drive.getHubDistance());
        double forwardPowerFwdPos = controls.getForwardPowerFwdPositive();
        double strafePowerLeftPos = controls.getStrafePowerLeftPositive();
        double rotatePowerCcwPos = controls.getRotatePowerCcwPositive();

        Transform2d robotVel = new Transform2d(forwardPowerFwdPos, strafePowerLeftPos, new Rotation2d(rotatePowerCcwPos));

        if (controls.getLeftAdjustReleased()) {
            shooter.nudgeAim(controls.getRightAdjust(), Translation2d.kZero);
        } else if (controls.getRightAdjustReleased()) {
            shooter.nudgeAim(Translation2d.kZero, controls.getRightAdjust());
        }

        shooter.autoAdjust(shootReady, robotVel, controls.getLeftAdjust(), controls.getFieldDrive(), true);

        if (reverseIndexer) {
            hopper.reverse();
        } else if (shootButton) {
            hopper.indexFuel();
        } else {
            hopper.stopMotors();
        }
    }

    public void grabberControl() {
        boolean intakeUp = controls.getIntakeUp();
        boolean intakeDown = controls.getIntakeDown();
        boolean runIntake = controls.getRunIntake();
        boolean reverseIntake = controls.getReverseIntake();

        if (runIntake) {
            grabber.intake();
        } else if (reverseIntake) {
            grabber.outtake();
        } else {
            grabber.stopWheel();
        }

        if (intakeDown) {
            grabber.lowerGrabber();
        } else if (intakeUp) {
            grabber.raiseGrabber();
        } else {
            grabber.stopGrabber();
        }
    }

    public void elasticControl() {
        PositionState currentPositionState = Drive.getPositionState();

        if (currentPositionState == Drive.PositionState.AWAY) {
            Elastic.selectTab("Defense");
        } else {
            if (AllianceUtil.isEndgame(DriverStation.getMatchTime())) {
                Elastic.selectTab("Endgame");
            } else {
                if (AllianceUtil.isRedAlliance()) {
                    Elastic.selectTab("Teleop (Red)");
                } else {
                    Elastic.selectTab("Teleop (Blue)");
                }
            }
        }
    }
    
    /*********************************/
    /**********TEST PROGRAMS**********/
    /*********************************/

    public void testShooterControl() {
        boolean shootReady = controls.getPauseTurret();
        boolean shootButton = controls.getShootButton();
        // boolean atTargetRPM = shooter.atTargetRPM();

        // shooter.autoAdjust(false);

        // we always run the flywheel but the hood should be down if we aren't ready
        if (shootReady) {
            shooter.setTargetRPMs(SmartDashboard.getNumber("TargetRightWheelRPM", 0), SmartDashboard.getNumber("TargetLeftWheelRPM", 0));
            shooter.setHoodAngle(SmartDashboard.getNumber("TargetLeftHoodAngleDegrees", 0), SmartDashboard.getNumber("TargetRightHoodAngleDegrees", 0));
        } else {
            shooter.stopWheels();
            hopper.stopMotors();
        }

        if (shootButton && shootReady) {
            hopper.indexFuel();
        } else {
            hopper.stopMotors();
        }
    }
}
