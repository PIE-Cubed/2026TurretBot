package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class Constants {

    public static class FieldConstants {

        public static final int TELEOP_LENGTH_SECONDS = 140;
        public static final int TRANSITION_PERIOD_SECONDS = 10;
        public static final int SHIFT_LENGTH_SECONDS = 25;
        public static final int ENDGAME_LENGTH_SECONDS = 30;

        public static final String aprilTagJson = "2026-rebuilt-welded";
        // public static final Path aprilTagJsonPath = Path.of(
        //     Filesystem.getDeployDirectory().getPath(),
        //     "apriltags",
        //     aprilTagJson + ".json"
        // );

        public static AprilTagFieldLayout aprilTagLayout;

        // static {
        //     try {
        //         aprilTagLayout = new AprilTagFieldLayout(aprilTagJsonPath);
        //     } catch (IOException e) {
        //         throw new RuntimeException(e);
        //     }
        // }

        public static final Pose2d hubBlueAlliance = new Pose2d(
            4.626081466674805,
            4.034362316131592,
            Rotation2d.kZero
        );
        public static final Pose2d hubRedAlliance = new Pose2d(
            11.916338920593262,
            4.034362316131592,
            Rotation2d.kZero
        );

        // top of the plastic ring on the hub is 72 inches
        public static final Distance hubHeight = Units.Inches.of(72 - 8);
    }
}
