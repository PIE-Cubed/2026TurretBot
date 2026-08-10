// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ADRC;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * A positional controller using ADRC.
 */
public class ADRCPositionController {
    private double outputClamp = 0;
    private double bGain = 0;

    private double observerGain1 = 0;
    private double observerGain2 = 0;
    private double observerGain3 = 0;

    private double pGain = 0;
    private double dGain = 0;

    private double lastOutput = 0;
    private double estimatedPosition = 0;
    private double estimatedVelocity = 0;
    private double estimatedDisturbance = 0;

    private Timer timer;

    public ADRCPositionController(double bGain, double observerBandwidth, double controlBandwidth, double outputClamp) {
        this.outputClamp = outputClamp;
        this.bGain = bGain;

        this.observerGain1 = 3 * observerBandwidth;
        this.observerGain2 = 3 * observerBandwidth * observerBandwidth;
        this.observerGain3 = observerBandwidth * observerBandwidth * observerBandwidth;

        this.pGain = controlBandwidth * controlBandwidth;
        this.dGain = 2 * controlBandwidth;

        timer = new Timer();
    }

    /**
     * Calculates motor output based on a position measurement and target.
     * @param measuredPosition The position measurement.
     * @param targetPosition The target position.
     * @return The output to be applied to the motor.
     */
    public double calculate(double measuredPosition, double targetPosition) {
        double error = measuredPosition - estimatedPosition;
        double dT = timer.get();

        estimatedPosition += dT * (estimatedVelocity + observerGain1 * error);
        estimatedVelocity += dT * (estimatedDisturbance + (bGain * lastOutput) + (observerGain2 * error));
        estimatedDisturbance += dT * (observerGain3 * error);

        double desiredAccel = pGain * (targetPosition - estimatedPosition) + (dGain * estimatedVelocity);
        double output = (desiredAccel - estimatedDisturbance) / bGain;
        output = MathUtil.clamp(output, -outputClamp, outputClamp);

        lastOutput = output;
        timer.restart();
        return output;
    }

    /**
     * Resets internal variables. 
     * Useful for making sure your calculations aren't out of whack if {@code calculate()} isn't called for a loop or more.
     * <p> TODO check if not calling this during teleopInit() or autonomousInit() messes up the first calculation
     */
    public void reset() {
        lastOutput = 0;

        estimatedPosition = 0;
        estimatedVelocity = 0;
        estimatedDisturbance = 0;

        timer.restart();
    }

    /**
     * Sets the tuning values for the ADRC Controller.
     */
    public void setValues(double bGain, double observerBandwidth, double controlBandwidth) {
        this.bGain = bGain;

        this.observerGain1 = 3 * observerBandwidth;
        this.observerGain2 = 3 * observerBandwidth * observerBandwidth;
        this.observerGain3 = observerBandwidth * observerBandwidth * observerBandwidth;

        this.pGain = controlBandwidth * controlBandwidth;
        this.dGain = 2 * controlBandwidth;

        reset();
    }
}