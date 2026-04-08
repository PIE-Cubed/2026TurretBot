package frc.robot.util;
 
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
 
/**
 * Wraps WPILib's KalmanFilter for 1D angle estimation.
 *
 * State:   x = [angle]
 * Input:   u = [0]   (no control input — pure observer)
 * Output:  y = [angle measurement]
 *
 * Tuning:
 *   MODEL_STD_DEV       — how much you trust your model (larger = less trust = reacts faster)
 *   MEASUREMENT_STD_DEV — how much you trust your sensor (larger = less trust = smoother)
 */
public class AngleKalmanFilter {
 
    // --- Tune these ---
    private static final double MODEL_STD_DEV       = 3.0;  // Q: model uncertainty
    private static final double MEASUREMENT_STD_DEV = 0.35; // R: sensor noise
 
    private static final double DT = 0.020; // 20ms — standard FRC loop period
 
    // Identity system: x_next = x, y = x (angle passes straight through)
    private final LinearSystem<N1, N1, N1> plant = new LinearSystem<>(
        MatBuilder.fill(Nat.N1(), Nat.N1(), 1.0), // A: state transition (angle stays the same)
        MatBuilder.fill(Nat.N1(), Nat.N1(), 0.0), // B: no control input
        MatBuilder.fill(Nat.N1(), Nat.N1(), 1.0), // C: output = state
        MatBuilder.fill(Nat.N1(), Nat.N1(), 0.0)  // D: no feedthrough
    );
 
    private final KalmanFilter<N1, N1, N1> filter = new KalmanFilter<>(
        Nat.N1(),
        Nat.N1(),
        plant,
        VecBuilder.fill(MODEL_STD_DEV),
        VecBuilder.fill(MEASUREMENT_STD_DEV),
        DT
    );
 
    /**
     * Call this every loop with your raw angle measurement.
     *
     * @param measurement raw angle (degrees or radians — stay consistent)
     * @return filtered angle estimate
     */
    public double update(double measurement) {
        filter.predict(VecBuilder.fill(0.0), DT); // No control input
        filter.correct(VecBuilder.fill(0.0), VecBuilder.fill(measurement));
        return filter.getXhat(0);
    }
 
    /** Returns the current filtered angle without updating. */
    public double getEstimate() {
        return filter.getXhat(0);
    }
 
    /** Reset the filter state (e.g. on robot enable). */
    public void reset() {
        filter.reset();
    }
}