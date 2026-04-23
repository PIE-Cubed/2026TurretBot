// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.Choreo;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Logger;

import java.util.List;
import java.util.Optional;

/**
 * This class has all the autonomous programs.
 */
public class Auto {

    // Required classes
    private Drive drive;
    // private Climber climber;
    private Shooter shooter;
    private Hopper hopper;
    private Grabber grabber;

    // Class-wide variables
    private final Timer timer = new Timer();
    private final Timer waitTimer = new Timer();
    // private final Timer waitTimer2 = new Timer();

    private Optional<Trajectory<SwerveSample>> centerOP1;
    private Optional<Trajectory<SwerveSample>> centerOP2;
    private Optional<Trajectory<SwerveSample>> centerDP1;
    private Optional<Trajectory<SwerveSample>> centerDP2;
    private Optional<Trajectory<SwerveSample>> depotPass2;
    private Optional<Trajectory<SwerveSample>> depotV1;
    private Optional<Trajectory<SwerveSample>> depotV2;
    private Optional<Trajectory<SwerveSample>> outpostNC;
    private Optional<Trajectory<SwerveSample>> outpostV1;
    private Optional<Trajectory<SwerveSample>> outpostV2;
    private Optional<Trajectory<SwerveSample>> outpostPass2;
    private Optional<Trajectory<SwerveSample>> testAuto;
    // private Optional<Trajectory<SwerveSample>> currentSplit;
    // private int currentSplitIndex = 1;
    private boolean firstTime = true;
    private int step = 1;

    // Auto-specific variables
    // Outpost
    // private int outpostClimbOutStep = 1;
    // private int outpostClimbDepStep = 1;
    // private int outpostNoClimbStep = 1;

    // Center
    // private int centerOutClimbOutStep = 1;
    // private int centerDepClimbOutStep = 1;
    // private int centerOutClimbDepStep = 1;
    // private int centerDepClimbDepStep = 1;
    // private int centerOutNoClimbStep = 1;
    // private int centerDepNoClimbStep = 1;

    // Depot
    // private int depotClimbOutStep = 1;
    // private int depotClimbDepStep = 1;
    // private int depotNoClimbStep = 1;

    @SuppressWarnings("unchecked")
    public Auto(Drive drive, Shooter shooter, Hopper hopper, Grabber grabber) {
        this.drive = drive;
        this.shooter = shooter;
        this.hopper = hopper;
        // this.climber = climber;
        this.grabber = grabber;

        centerOP1 = Choreo.loadTrajectory("centerOV1P1");
        centerOP2 = Choreo.loadTrajectory("centerOV1P2");
        centerDP1 = Optional.of(((Trajectory<SwerveSample>) Choreo.loadTrajectory("centerOV1P1").get()).mirrorY());
        centerDP2 = Optional.of(((Trajectory<SwerveSample>) Choreo.loadTrajectory("centerOV1P1").get()).mirrorY());
        // outpostNC = Choreo.loadTrajectory("holyFemale");
        outpostV1 = Choreo.loadTrajectory("outpostV1");
        outpostV2 = Choreo.loadTrajectory("outpostV2");
        depotV1 = Optional.of(((Trajectory<SwerveSample>) Choreo.loadTrajectory("outpostV1").get()).mirrorY());
        depotV2 = Optional.of(((Trajectory<SwerveSample>) Choreo.loadTrajectory("outpostV2").get()).mirrorY());
        outpostPass2 = Choreo.loadTrajectory("outpostV1P2");
        depotPass2 = Optional.of(((Trajectory<SwerveSample>) Choreo.loadTrajectory("outpostV1P2").get()).mirrorY());
    }

    public void resetAuto() {
        firstTime = true;
    }

    ///////////////////
    /* OUTPOST AUTOS */
    ///////////////////

    public int crazyOutpostAuto() {
        if (firstTime) {
            firstTime = false;
            step = 1;
            restartTimer();
            waitTimer.restart();
        }

        int status = Robot.CONT;
        int pathStatus = choreoPathFollower(outpostNC);

        // SmartDashboard.putNumber("Auto Step", step);

        boolean hoodUp = false;

        switch (step) {
            case 1:
                hoodUp = false;

                grabber.lowerGrabber();

                status = atMarker(outpostNC, "start intake");
                break;
            case 2:
                hoodUp = true;

                grabber.lowerGrabber();
                grabber.intake();

                status = atMarker(outpostNC, "start pass");
                break;
            case 3:
                hoodUp = true;

                grabber.stopGrabber();
                grabber.intake();
                hopper.indexFuel();

                status = atMarker(outpostNC, "stop pass");
                break;
            case 4:
                hoodUp = true;

                grabber.intake();
                hopper.reverse();

                status = atMarker(outpostNC, "start shoot");
                break;
            case 5:
                hoodUp = true;

                grabber.intake();
                hopper.indexFuel();

                status = atMarker(outpostNC, "stop shoot");
                break;
            case 6:
                hoodUp = false;

                grabber.intake();
                hopper.stopMotors();

                status = pathStatus;
                break;
            default:
                drive.stopWheels();
                return Robot.DONE;
        }

        SwerveSample currSample = (SwerveSample) Logger.getStruct("choreoSample", SwerveSample.struct).get();

        if (currSample == null) {
            currSample = new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[] {0, 0, 0, 0}, new double[] {0, 0, 0, 0});
        }

        shooter.autoAdjust(hoodUp, 
            new Transform2d(currSample.vx, currSample.vy, Rotation2d.fromRadians(currSample.omega)), 
            Translation2d.kZero,
            true
        );

        if (status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    public int outpostAuto(boolean mod2) {
        if (firstTime) {
            firstTime = false;
            step = 1;
            restartTimer();
            waitTimer.restart();
        }

        int status = Robot.CONT;

        // SmartDashboard.putNumber("Auto Step", step);

        boolean hoodUp = false;

        switch (step) {
            case 1:
                choreoPathFollower((mod2) ? outpostV2 : outpostV1);
                hoodUp = false;

                grabber.lowerGrabber();

                status = atMarker((mod2) ? outpostV2 : outpostV1, "start intake");
                break;
            case 2:
                choreoPathFollower((mod2) ? outpostV2 : outpostV1);
                hoodUp = false;

                grabber.lowerGrabber();
                grabber.intake();

                status = atMarker((mod2) ? outpostV2 : outpostV1, "start shoot");
                break;
            case 3:
                choreoPathFollower((mod2) ? outpostV2 : outpostV1);
                hoodUp = true;

                grabber.stopGrabber();
                grabber.intake();
                hopper.indexFuel();

                status = atMarker((mod2) ? outpostV2 : outpostV1, "stop shoot");
                break;
            case 4:
                timer.restart();

                status = Robot.CONT;
                break;
            case 5:
                choreoPathFollower(outpostPass2);
                hoodUp = false;

                grabber.intake();
                grabber.lowerGrabber();
                hopper.reverse();

                status = atMarker(outpostPass2, "start shoot");
                break;
            case 6:
                choreoPathFollower(outpostPass2);
                hoodUp = true;

                grabber.intake();
                grabber.stopGrabber();
                hopper.indexFuel();

                if (DriverStation.isFMSAttached()) {
                    status = Robot.CONT;
                } else {
                    status = (DriverStation.getMatchTime() <= 0 || DriverStation.getMatchTime() >= 20) ? Robot.DONE : Robot.CONT;
                }
                break;
            default:
                drive.stopWheels();
                hopper.stopMotors();
                grabber.stopGrabber();
                grabber.stopWheel();
                shooter.stopHood();
                shooter.stopTurrets();
                shooter.stopWheels();
                return Robot.DONE;
        }

        shooter.autoAdjust(hoodUp);

        if (status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    //////////////////
    /* CENTER AUTOS */
    //////////////////

    public int centerDepClimbDep() {
        return Robot.CONT;
    }

    public int centerOut(boolean climb) {
        if (firstTime) {
            firstTime = false;
            step = 1;
            restartTimer();
            waitTimer.restart();
        }

        int status = Robot.CONT;

        // SmartDashboard.putNumber("Auto Step", step);

        boolean hoodUp = false;

        switch (step) {
            case 1:
                choreoPathFollower(centerOP1);
                hoodUp = false;

                grabber.lowerGrabber();

                status = atMarker(centerOP1, "start intake");
                break;
            case 2:
                choreoPathFollower(centerOP1);
                hoodUp = false;

                grabber.lowerGrabber();
                grabber.intake();

                status = atMarker(centerOP1, "start shoot");
                break;
            case 3:
                choreoPathFollower(centerOP1);
                hoodUp = true;

                grabber.stopGrabber();
                grabber.intake();
                hopper.indexFuel();

                status = atMarker(centerOP1, "stop shoot");
                break;
            case 4:
                timer.restart();

                status = Robot.CONT;
                break;
            case 5:
                choreoPathFollower(centerOP1);
                hoodUp = true;

                grabber.intake();
                grabber.lowerGrabber();
                hopper.reverse();

                status = atMarker(centerOP1, "start shoot");
                break;
            case 6:
                choreoPathFollower(centerOP1);
                hoodUp = true;

                grabber.intake();
                grabber.stopGrabber();
                hopper.indexFuel();

                if (DriverStation.isFMSAttached()) {
                    status = Robot.CONT;
                } else {
                    status = (DriverStation.getMatchTime() <= 0 || DriverStation.getMatchTime() >= 20) ? Robot.DONE : Robot.CONT;
                }
                break;
            default:
                drive.stopWheels();
                hopper.stopMotors();
                grabber.stopGrabber();
                grabber.stopWheel();
                shooter.stopHood();
                shooter.stopTurrets();
                shooter.stopWheels();
                return Robot.DONE;
        }

        shooter.autoAdjust(hoodUp);

        if (status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    public int testAuto() {
        if (firstTime) {
            System.out.println("First time");
            firstTime = false;
            step = 1;
            restartTimer();
        }

        System.out.println("Running");

        int status = Robot.CONT;
        choreoPathFollower(testAuto);

        switch (step) {
            case 1:
                System.out.println("Step 1");
                status = atMarker(testAuto, "test marker");

                break;
            default:
                drive.stopWheels();
                return Robot.DONE;
        }

        if (status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    public int centerDepNoClimb() {
        return Robot.CONT;
    }

    public int centerOutNoClimb() {
        return Robot.CONT;
    }

    /////////////////
    /* DEPOT AUTOS */
    /////////////////

    public int depotAuto(boolean mod2) {
        if (firstTime) {
            firstTime = false;
            step = 1;
            restartTimer();
            waitTimer.restart();
        }

        int status = Robot.CONT;

        // SmartDashboard.putNumber("Auto Step", step);

        boolean hoodUp = false;

        switch (step) {
            case 1:
                choreoPathFollower((mod2) ? depotV2 : depotV1);
                hoodUp = false;

                grabber.lowerGrabber();

                status = atMarker((mod2) ? depotV2 : depotV1, "start intake");
                break;
            case 2:
                choreoPathFollower((mod2) ? depotV2 : depotV1);
                hoodUp = false;

                grabber.lowerGrabber();
                grabber.intake();

                status = atMarker((mod2) ? depotV2 : depotV1, "start shoot");
                break;
            case 3:
                choreoPathFollower((mod2) ? depotV2 : depotV1);
                hoodUp = true;

                grabber.stopGrabber();
                grabber.intake();
                hopper.indexFuel();

                status = atMarker((mod2) ? depotV2 : depotV1, "stop shoot");
                break;
            case 4:
                timer.restart();

                status = Robot.CONT;
                break;
            case 5:
                choreoPathFollower(depotPass2);
                hoodUp = false;

                grabber.intake();
                grabber.lowerGrabber();
                hopper.reverse();

                status = atMarker(depotPass2, "start shoot");
                break;
            case 6:
                choreoPathFollower(depotPass2);
                hoodUp = true;

                grabber.intake();
                grabber.stopGrabber();
                hopper.indexFuel();

                if (DriverStation.isFMSAttached()) {
                    status = Robot.CONT;
                } else {
                    status = (DriverStation.getMatchTime() <= 0 || DriverStation.getMatchTime() >= 20) ? Robot.DONE : Robot.CONT;
                }
                break;
            default:
                drive.stopWheels();
                hopper.stopMotors();
                grabber.stopGrabber();
                grabber.stopWheel();
                shooter.stopHood();
                shooter.stopTurrets();
                shooter.stopWheels();
                return Robot.DONE;
        }

        shooter.autoAdjust(hoodUp);

        if (status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /////////////
    /* HELPERS */
    /////////////

    public int choreoPathFollower(Optional<Trajectory<SwerveSample>> trajectory) {
        // if (currentTrajectory.get().name() != trajectoryFileName) {
        //     currentTrajectory = Choreo.loadTrajectory(trajectoryFileName);
        // }

        // if (currentSplitIndex != splitIndex) {
        //     currentSplit = currentTrajectory.get().getSplit(splitIndex);
        //     currentSplitIndex = splitIndex;
        // }

        if (trajectory.isPresent()) {
            Optional<SwerveSample> sample = trajectory.get().mirrorY().sampleAt(timer.get(), AllianceUtil.isRedAlliance());

            if (sample.isPresent()) {
                //System.out.println("Sample time: " + sample.get().getTimestamp());

                drive.followSample(sample.get());
                // int inTolerance = drive.followSample(sample.get());

                // Tolerances are most likely unnecessary here.
                if (
                    // inTolerance == Robot.DONE &&
                    sample.get().t == trajectory.get().getFinalSample(AllianceUtil.isRedAlliance()).get().t
                ) {
                    return Robot.DONE;
                }
                // choreoPose2d = sample.get().getPose();
                Logger.logStruct("choreoSample", sample.get());
            }
        } else { // there isn't a trajectory
            System.err.println("ERROR 404: Trajectory not found");
            return Robot.FAIL;
        }

        return Robot.CONT;
    }

    /**
     * Checks whether the choreo path is at a marker.
     * Uses recursion, so try not to run trajectories with more than a couple markers that have the same name.
     * @param markerName The name of the marker.
     * @return Robot.DONE if at a marker, Robot.CONT if not.
     */
    public int atMarker(Optional<Trajectory<SwerveSample>> trajectory, String markerName) {
        List<EventMarker> events = trajectory.get().getEvents(markerName);

        for (int i = 0; i < events.size(); i++) {
            if (MathUtil.isNear(events.get(i).timestamp, timer.get(), 0.06)) return Robot.DONE;
        }

        return Robot.CONT;
    }

    public void restartTimer() {
        timer.restart();
    }
}
