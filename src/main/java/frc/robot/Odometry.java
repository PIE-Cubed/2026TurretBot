package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class Odometry {

    public static List<PhotonPipelineResult> cameraResults;
    public static PhotonPipelineResult latestResult;

    private final int REQUIRED_APRILTAGS = 2; // Number of required AprilTags to update the AprilTag estimator
    private final double MAX_YAW_RATE_DEGREES = 360; // Maximum angular velocity(degrees/s) to update AprilTag estimator
    private final double AMBIGUITY_CUTOFF = 0.1;

    /***********************************************************************************************************************/
    /*                Standard deviations for each camera, with X and Y in meters and rotation in radians.                 */
    /***********************************************************************************************************************/
    /*                                                                   | X |     | Y |          | ROTATION |             */
    public static final Vector<N3> CAMERA1_STD_DEVS = VecBuilder.fill(0.125, 0.125, Units.degreesToRadians(1));
    public static final Vector<N3> CAMERA2_STD_DEVS = VecBuilder.fill(0.125, 0.125, Units.degreesToRadians(1));
    public static final Vector<N3> CAMERA3_STD_DEVS = VecBuilder.fill(0.125, 0.125, Units.degreesToRadians(1));
    public static final Vector<N3> CAMERA4_STD_DEVS = VecBuilder.fill(0.125, 0.125, Units.degreesToRadians(1));

    // Distances from bottom center of robot to each camera
    // When rotation is 0 for all axes the Z axis is parallel to the front of the robot.
    // TODO: get camera offsets for new spots
    private final Transform3d ROBOT_TO_CAMERA1 = new Transform3d(
        Units.inchesToMeters(-11.75),
        Units.inchesToMeters(-11.75),
        Units.inchesToMeters(15.111479),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-21.25), Units.degreesToRadians(-150))
    );

    private final Transform3d ROBOT_TO_CAMERA2 = new Transform3d(
        Units.inchesToMeters(-11.75),
        Units.inchesToMeters(11.75),
        Units.inchesToMeters(15.111479),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-21.25), Units.degreesToRadians(120))
    );

    private final Transform3d ROBOT_TO_CAMERA3 = new Transform3d(
        Units.inchesToMeters(-4.0),
        Units.inchesToMeters(-9.75), 
        Units.inchesToMeters(20.0), 
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(-20))
    );

    private final Transform3d ROBOT_TO_CAMERA4 = new Transform3d(
        Units.inchesToMeters(-4.0),
        Units.inchesToMeters(11.0), 
        Units.inchesToMeters(20.0), 
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-30), Units.degreesToRadians(0))
    );

    // Vision estimators
    private PhotonPoseEstimator camera1PoseEstimator; // Photon Vision estimators
    private PhotonPoseEstimator camera2PoseEstimator; //
    private PhotonPoseEstimator camera3PoseEstimator; //
    private PhotonPoseEstimator camera4PoseEstimator; //

    // Photon Vision cameras
    private static PhotonCamera camera1;
    private List<PhotonPipelineResult> camera1Results;
    private static PhotonCamera camera2;
    private List<PhotonPipelineResult> camera2Results;
    private static PhotonCamera camera3;
    private List<PhotonPipelineResult> camera3Results;
    private static PhotonCamera camera4;
    private List<PhotonPipelineResult> camera4Results;

    private CameraPackage camera1Package;
    private CameraPackage camera2Package;
    private CameraPackage camera3Package;
    private CameraPackage camera4Package;

    private EstimatedRobotPose camera1RobotPose;
    private EstimatedRobotPose camera2RobotPose;
    private EstimatedRobotPose camera3RobotPose;
    private EstimatedRobotPose camera4RobotPose;

    public Drive drive;

    public Odometry(Drive drive) {
        this.drive = drive;

        // Instantiate the PhotonCameras
        camera1 = new PhotonCamera("BRCamera");
        camera2 = new PhotonCamera("BLCamera");
        camera3 = new PhotonCamera("FRCamera");
        camera4 = new PhotonCamera("FLCamera");

        // Instantiate the pose estimators for each camera
        camera1PoseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark), // Field selection
            ROBOT_TO_CAMERA1 // Camera offset from robot
        );

        camera2PoseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
            ROBOT_TO_CAMERA2
        );
        
        camera3PoseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
            ROBOT_TO_CAMERA3
        );

        camera4PoseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
            ROBOT_TO_CAMERA4
        );

        camera1Package = new CameraPackage(camera1, camera1PoseEstimator, AMBIGUITY_CUTOFF, MAX_YAW_RATE_DEGREES);
        camera2Package = new CameraPackage(camera2, camera2PoseEstimator, AMBIGUITY_CUTOFF, MAX_YAW_RATE_DEGREES);
        camera3Package = new CameraPackage(camera3, camera3PoseEstimator, AMBIGUITY_CUTOFF, MAX_YAW_RATE_DEGREES);
        camera4Package = new CameraPackage(camera4, camera4PoseEstimator, AMBIGUITY_CUTOFF, MAX_YAW_RATE_DEGREES);
    }

    /**
     * <p> Updates the pose estimators. Calling this outside of robotPeriodic is unnecesary.
     * <p> Call once per loop.
     */
    public void updateVisionEstimators() {
        int tagCount = 0;

        tagCount += camera1Package.updateCamera(drive.getYawRateDegrees());
        tagCount += camera2Package.updateCamera(drive.getYawRateDegrees());
        tagCount += camera3Package.updateCamera(drive.getYawRateDegrees());
        tagCount += camera4Package.updateCamera(drive.getYawRateDegrees());

        camera1RobotPose = camera1Package.getPose();
        camera2RobotPose = camera2Package.getPose();
        camera3RobotPose = camera3Package.getPose();
        camera4RobotPose = camera4Package.getPose();

        if (tagCount < REQUIRED_APRILTAGS) {
            camera1RobotPose = null;
            camera2RobotPose = null;
            camera3RobotPose = null;
            camera4RobotPose = null;
        }
    }

    /**
     * </p> Gets the current AprilTag-assisted field position from camera1.
     *      If no tags are seen or the Orange PIs haven't produced a new result, returns null.
     * </p> Refer to the WPILib docs for specifics on field-based odometry.
     *
     * @return The estimated Pose. (in meters)
     */
    public EstimatedRobotPose getCamera1Pose() {
        return camera1RobotPose;
    }

    /**
     * </p> Gets the current AprilTag-assisted field position from camera2.
     *      If no tags are seen or the Orange PIs haven't produced a new result, returns null.
     * </p> Refer to the WPILib docs for specifics on field-based odometry.
     *
     * @return The estimated Pose. (in meters)
     */
    public EstimatedRobotPose getCamera2Pose() {
        return camera2RobotPose;
    }

    /**
     * </p> Gets the current AprilTag-assisted field position from camera3.
     *      If no tags are seen or the Orange PIs haven't produced a new result, returns null.
     * </p> Refer to the WPILib docs for specifics on field-based odometry.
     *
     * @return The estimated Pose. (in meters)
     */
    public EstimatedRobotPose getCamera3Pose() {
        return camera3RobotPose;
    }

    /**
     * </p> Gets the current AprilTag-assisted field position from camera4.
     *      If no tags are seen or the Orange PIs haven't produced a new result, returns null.
     * </p> Refer to the WPILib docs for specifics on field-based odometry.
     *
     * @return The estimated Pose. (in meters)
     */
    public EstimatedRobotPose getCamera4Pose() {
        return camera4RobotPose;
    }

    /**
     * @return
     * <p> Distance from the camera to the nearest/best target.
     * <p> X = forward, Y = left, Z = up. Distance is measured in meters.
     * <p> Rotation (when measured in degrees) stretches from -180 to 180, when the tag is parallel to the camera the angle is 0.
     */
    public Transform3d getAprilTagDistanceToCameraMetric() {
        if (latestResult == null) {
            return null;
        }

        if (!latestResult.hasTargets()) {
            return null;
        }

        Transform3d nonRotated = latestResult.getBestTarget().getBestCameraToTarget().inverse();

        return new Transform3d(
            nonRotated.getTranslation(),
            nonRotated.getRotation().rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(180)))
        );
    }

    /**
     * @return
     * <p> Distance from the camera to the nearest/best target.
     * <p> X = forward, Y = left, Z = up. Distance is measured in inches.
     * <p> Rotation (when measured in degrees) stretches from -180 to 180, when the tag is parallel to the camera the angle is 0.
     */
    public Transform3d getAprilTagDistanceToCamera() {
        Transform3d metricTransform = getAprilTagDistanceToCameraMetric();

        if (metricTransform == null) {
            return null;
        }

        return new Transform3d(
            (Units.metersToInches(metricTransform.getX())),
            (Units.metersToInches(metricTransform.getY())),
            (Units.metersToInches(metricTransform.getZ())),
            metricTransform.getRotation()
        );
    }

    /**
     * @return
     * Distance from the robot to the nearest/best target (camera POV).
     * X = forward, Y = left, Z = up. Distance is measured in meters.
     * Rotation (when measured from degrees) stretches from -180 to 180, when the tag is parallel to the camera the angle is 0.
     */
    public Transform3d getAprilTagDistanceMetric() {
        if (getAprilTagDistanceToCameraMetric() == null) {
            return null;
        }

        return getAprilTagDistanceToCameraMetric().plus(ROBOT_TO_CAMERA1.inverse());
    }

    /**
     * @return
     * Distance from the robot to the nearest/best target (camera POV).
     * X = forward, Y = left, Z = up. Distance is measured in inches.
     * Rotation (when measured from degrees) stretches from -180 to 180, when the tag is parallel to the camera the angle is 0.
     */
    public Transform3d getAprilTagDistance() {
        if (getAprilTagDistanceToCamera() == null) {
            return null;
        }

        return getAprilTagDistanceToCamera().plus(ROBOT_TO_CAMERA1.inverse());
    }

    /**
     * @return
     * The rotation of the robot relative to the tag (0 is parallel to the tag & positive is clockwise)
     */
    public Rotation2d robotToTagRotation() {
        if (getAprilTagDistanceMetric() == null) {
            return null;
        }

        return getAprilTagDistanceMetric().getRotation().toRotation2d();
    }

    /**
     * This should be tested at some point (if used)
     * @return
     * The rotation of the robot relative to the tag (0 is pointing at the tag & positive is clockwise)
     */
    public Rotation2d robotPointingToTagRotation() {
        if (getAprilTagDistance() == null) {
            return null;
        }

        // photonVision doesn't give this as a function (unlike the limelight) and we need
        // to calculate it manually via trigonometry
        double theta = Math.atan2(getAprilTagDistance().getY(), getAprilTagDistance().getX());

        return new Rotation2d(theta);
    }

    /**
     * Gets every tag seen by each camera. May contain duplicates of the same tag.
     * @return A list of each tag seen by each camera.
     */
    public List<PhotonTrackedTarget> getAllTags() {
        ArrayList<PhotonTrackedTarget> allTags = new ArrayList<PhotonTrackedTarget>();

        allTags.addAll(camera1Results.get(camera1Results.size() - 1).getTargets());
        allTags.addAll(camera2Results.get(camera2Results.size() - 1).getTargets());
        allTags.addAll(camera3Results.get(camera3Results.size() - 1).getTargets());
        allTags.addAll(camera4Results.get(camera4Results.size() - 1).getTargets());

        return (List<PhotonTrackedTarget>) allTags;
    }

    /**
     * @return
     * The fiducial ID of the primary april tag.
     */
    public int primaryTagID() {
        return latestResult.getBestTarget().getFiducialId();
    }

    /**
     * @return
     * How many april tags the camera sees.
     */
    public int targetCount() {
        return latestResult.getTargets().size();
    }

    /**
     * @return
     * Whether the camera sees any targets.
     */
    public boolean hasTargets() {
        return !getAllTags().isEmpty();
    }

    /*******************************************************************************************
     *
     *                                     HELPER FUNCTIONS
     *
     *******************************************************************************************/

    /*******************************************************************************************
     *
     *                                      TEST FUNCTIONS
     *
     *******************************************************************************************/

    public static class CameraPackage {
        private PhotonCamera camera;
        private PhotonPoseEstimator poseEstimator;

        private double ambiguityCutoff;
        private double maxYawRateDegrees;

        private EstimatedRobotPose currentPose;
        private List<PhotonPipelineResult> cameraResults;

        public CameraPackage(
            PhotonCamera camera, 
            PhotonPoseEstimator poseEstimator, 
            double ambiguityCutoff, 
            double maxYawRateDegrees
        ) {
            this.camera = camera;
            this.poseEstimator = poseEstimator;
            this.ambiguityCutoff = ambiguityCutoff;
            this.maxYawRateDegrees = maxYawRateDegrees;
        }

        public int updateCamera(double currentYawRateDegrees) {
            cameraResults = camera.getAllUnreadResults();

            int tagCount = 0;

            if (currentYawRateDegrees > maxYawRateDegrees) {
                currentPose = null;
            }

            if (cameraResults != null) { // If the camera hasn't connected yet this can be null
                if (!cameraResults.isEmpty()) { // Has the camera processed any new results?
                    PhotonPipelineResult newResult = cameraResults.get(cameraResults.size() - 1); // Getting newest result

                    latestResult = newResult;

                    if (newResult.hasTargets()) { // Does the camera see any targets?
                        tagCount += newResult.targets.size();

                        boolean allTargetsGood = true;

                        for (PhotonTrackedTarget target:newResult.targets) {
                            if (target.poseAmbiguity > ambiguityCutoff) {
                                allTargetsGood = false;
                                break;
                            }
                        }

                        // Get the estimated field pose from this camera
                        if (newResult.targets.size() >= 2 && allTargetsGood) {
                            currentPose = poseEstimator.estimateCoprocMultiTagPose(newResult).orElse(null);
                        } else {
                            if (newResult.getBestTarget().poseAmbiguity < ambiguityCutoff) {
                                currentPose = poseEstimator.estimateLowestAmbiguityPose(newResult).orElse(null);
                            }
                        }
                        // System.out.println("camera1RobotPose" + camera1RobotPose.estimatedPose);
                        // Logger.logStruct("camera1Results", camera1RobotPose.estimatedPose);
                    } else {
                        currentPose = null;
                    }
                } else {
                    currentPose = null;
                }
            }

            return tagCount;
        }

        public EstimatedRobotPose getPose() {
            return currentPose;
        }

        public List<PhotonPipelineResult> getRawResults() {
            return cameraResults;
        }
    }
}
