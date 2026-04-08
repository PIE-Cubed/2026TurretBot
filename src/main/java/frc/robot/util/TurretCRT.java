package frc.robot.util;

import edu.wpi.first.units.measure.Angle;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

/**
 * Absolute turret angle resolver using YAMS EasyCRT.
 *
 * Hardware:
 *   Turret ring   : 200 teeth  (1 mechanism rotation = 360°)
 *   Drive gear    : 21 teeth   (encoder 1 pinion meshes here)
 *   Encoder gear  : 19 teeth   (encoder 2 pinion meshes here)
 *
 * The turret ring IS the shared drive gear for both encoder pinions,
 * so commonRatio = 1.0 (one encoder-gear turn per one turret-ring turn... wait:
 *   actually the turret ring is the large gear; the drive motor meshes through
 *   the 21T pinion. The 21T and 19T pinions both mesh directly onto the 200T ring.
 *   So: one full turret rotation = 200/21 drive-encoder rotations
 *                                = 200/19 encoder-gear rotations.
 *   We express this as:
 *     enc1RotPerMechRot = 200.0 / 21.0
 *     enc2RotPerMechRot = 200.0 / 19.0
 *   and use withEncoderRatios() directly.
 *
 * Range: turret is a full 360° turret -> 1.0 rotation.
 *   Use a slightly negative lower bound so the wrap edge isn't at the parked pose
 *   (per YAMS debugging advice: use e.g. -0.1 to 0.9 instead of 0.0 to 1.0).
 *
 * Solve once at startup, then seed your motor controller. Do NOT call every loop.
 */
public class TurretCRT {

    // ── gear ratios ───────────────────────────────────────────────────────────
    // Both encoder pinions mesh directly onto the 200T turret ring.
    // enc rotations per one full turret rotation:
    private static final double ENC1_ROT_PER_MECH = 200.0 / 21.0; // ~9.524
    private static final double ENC2_ROT_PER_MECH = 200.0 / 19.0; // ~10.526

    private final EasyCRT solver;

    /**
     * @param enc1Supplier  Supplier returning the 21T-side encoder reading (Angle, wraps [0,1) rot)
     * @param enc2Supplier  Supplier returning the 19T-side encoder reading (Angle, wraps [0,1) rot)
     * @param enc1Offset    Encoder 1 raw reading at the physical 0° turret pose (Rotations)
     * @param enc2Offset    Encoder 2 raw reading at the physical 0° turret pose (Rotations)
     * @param enc1Inverted  True if encoder 1 reads backwards relative to turret positive direction
     * @param enc2Inverted  True if encoder 2 reads backwards relative to turret positive direction
     */
    public TurretCRT(
            Supplier<Angle> enc1Supplier,
            Supplier<Angle> enc2Supplier,
            double enc1Offset,
            double enc2Offset,
            boolean enc1Inverted,
            boolean enc2Inverted) {

        EasyCRTConfig config = new EasyCRTConfig(enc1Supplier, enc2Supplier)

            // Direct ratios: how many encoder rotations per one turret rotation
            .withEncoderRatios(ENC1_ROT_PER_MECH, ENC2_ROT_PER_MECH)

            // Offsets: raw encoder reading at your physical zero pose
            // Set these after mechanical zeroing. Do NOT also set them on-device.
            .withAbsoluteEncoderOffsets(
                Rotations.of(enc1Offset),
                Rotations.of(enc2Offset))

            // Inversion: set here as single source of truth.
            // Leave device-side inversion at defaults.
            .withAbsoluteEncoderInversions(enc1Inverted, enc2Inverted)

            // Mechanism range: use a slightly negative lower bound so the park pose
            // is never right at the wrap edge (YAMS debugging tip).
            // This covers a full rotation with margin on both sides.
            .withMechanismRange(Rotations.of(-0.1), Rotations.of(0.9))

            // Match tolerance: start around backlash mapped to encoder 2.
            // backlash_mech_rot * enc2RotPerMech. For ~0.5 tooth backlash:
            //   (0.5/200) * (200/19) ≈ 0.026 rotations. Use 0.04 for safety margin.
            // Tune down if you get AMBIGUOUS, up if you get NO_SOLUTION.
            .withMatchTolerance(Rotations.of(0.04));

        solver = new EasyCRT(config);
    }

    /**
     * Resolve the absolute turret angle. Call this ONCE at startup (robot init,
     * after encoders are ready) and use the result to seed your motor controller.
     * Do not call this every loop -- it has non-trivial CPU cost.
     *
     * @return the turret angle in Rotations, or empty if AMBIGUOUS / NO_SOLUTION
     */
    public java.util.Optional<Angle> resolveAbsoluteAngle() {
        return solver.getAngleOptional();
    }

    /**
     * Convenience: returns the last solve status for logging / SmartDashboard.
     * Values: OK, AMBIGUOUS, NO_SOLUTION
     */
    public String getStatus() {
        return solver.getLastStatus().toString();
    }

    /**
     * Returns the modular error from the last solve (rotations of encoder 2).
     * Use this to tune matchTolerance -- it should stay comfortably below your tolerance.
     */
    public double getLastErrorRotations() {
        return solver.getLastErrorRotations();
    }

    /**
     * Returns the number of candidate iterations the last solve required.
     * Log this to validate performance. Typical: tens of iterations.
     */
    public int getLastIterations() {
        return solver.getLastIterations();
    }


    // ── example usage in your turret subsystem ───────────────────────────────
    //
    //  private final TurretCRT crt = new TurretCRT(
    //      () -> enc1.getAbsolutePosition(),   // Supplier<Angle>
    //      () -> enc2.getAbsolutePosition(),   // Supplier<Angle>
    //      0.0,    // enc1Offset -- set after mechanical zero
    //      0.0,    // enc2Offset -- set after mechanical zero
    //      false,  // enc1Inverted
    //      false   // enc2Inverted
    //  );
    //
    //  @Override
    //  public void robotInit() {
    //      // Seed the motor controller once after boot, before closed-loop runs
    //      crt.resolveAbsoluteAngle().ifPresent(angle -> {
    //          turretMotor.setEncoderPosition(angle);
    //      });
    //      SmartDashboard.putString("CRT Status", crt.getStatus());
    //      SmartDashboard.putNumber("CRT Error (rot)", crt.getLastErrorRotations());
    //  }
}