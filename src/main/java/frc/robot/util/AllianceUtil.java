package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/**
 * REBUILT hub activation helper.
 *
 * matchTime behavior:
 * - Autonomous: counts down from ~20 to 0
 * - Teleop: counts down from ~140 to 0
 */
public final class AllianceUtil {

    // Prevent construction
    private AllianceUtil() {}

    // CONSTANTS
    private static final double TELEOP_LENGTH = 140.0;
    private static final double TELEOP_START_SECONDS = 10.0;
    private static final double ENDGAME_SECONDS = 30.0;

    // Four 25-second alternating windows during teleop
    private static final double[] WINDOW_STARTS = { 130.0, 105.0, 80.0, 55.0 };
    private static final double[] WINDOW_ENDS = { 105.0, 80.0, 55.0, 30.0 };

    /**
     * Checks if we are the red alliance
     *
     * @return isRedAlliance
     */
    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    public static boolean isEndgame(double matchTime) {
        if (matchTime <= ENDGAME_SECONDS) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Seconds until hub availability changes.
     *
     * @param matchTime
     * @return
     */
    public static double timeUntilHubStateChange(double matchTime) {
        if (DriverStation.isAutonomous()) return 0.0;
        if (isGuaranteedBothActive(matchTime)) return 0.0;

        int windowIndex = getWindowIndex(matchTime);
        if (windowIndex == -1) return 0.0;

        double endsAt = WINDOW_ENDS[windowIndex];
        return Math.max(0.0, matchTime - endsAt);
    }

    /**
     * True if OUR alliance hub is currently active.
     *
     * @param matchTime
     * @return
     */
    public static boolean isOurHubActive(double matchTime) {
        // Autonomous: always active for both alliances
        if (DriverStation.isAutonomous()) {
            return true;
        }

        // Guaranteed both-active windows during teleop
        if (isGuaranteedBothActive(matchTime)) {
            return true;
        }

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty()) {
            return true; // Fails open, no message provided
        }

        int windowIndex = getWindowIndex(matchTime);
        if (windowIndex == -1) {
            return true;
        }

        // FMS character is the alliance deactivated first
        char firstDeactivated = gameData.charAt(0);

        // Window 0: opposite alliance active first, then alternate
        char activeAlliance = (windowIndex % 2 == 0) ? oppositeAllianceChar(firstDeactivated) : firstDeactivated;

        boolean isRed = isRedAlliance();
        return (activeAlliance == 'R') == isRed;
    }

    private static boolean isGuaranteedBothActive(double matchTime) {
        // First 10 seconds of teleop: (140, 130]
        boolean first10 = matchTime <= TELEOP_LENGTH && matchTime > (TELEOP_LENGTH - TELEOP_START_SECONDS);

        // Endgame: [30,0]
        boolean endgame = matchTime <= ENDGAME_SECONDS;

        return first10 || endgame;
    }

    /**
     * Returns 0 to 3 for the current alternating window, otherwise -1.
     *
     * @param matchTime
     * @return
     */
    private static int getWindowIndex(double matchTime) {
        for (int i = 0; i < WINDOW_STARTS.length; i++) {
            // time <= start && time > end
            if (matchTime <= WINDOW_STARTS[i] && matchTime > WINDOW_ENDS[i]) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Gets the character of the opposite alliance
     *
     * @param allianceChar
     * @return
     */
    private static char oppositeAllianceChar(char allianceChar) {
        return (allianceChar == 'R') ? 'B' : 'R';
    }
}
