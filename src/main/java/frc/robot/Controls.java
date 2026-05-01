package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

public class Controls {

    XboxController manipController;
    ZorroController driveController;

    public static final double DRIVE_CONTROLLER_DEADZONE = 0.01;

    private boolean leftAdjusted = false;
    private boolean rightAdjusted = true;

    public Controls() {
        manipController = new XboxController(1);
        driveController = new ZorroController(0);
    }

    // Zorro Controller

    public double getForwardPowerFwdPositive() {
        double powerFwdPos = driveController.getLeftY();

        powerFwdPos = Math.pow(powerFwdPos, 3);

        if (enablePrecisionDrive()) {
            powerFwdPos *= 0.231;
        }

        return powerFwdPos;
    }

    public double getStrafePowerLeftPositive() {
        double powerRightPos = driveController.getLeftX();

        powerRightPos = Math.pow(powerRightPos, 3);

        if (enablePrecisionDrive()) {
            powerRightPos *= 0.231;
        }
        return powerRightPos * -1;
    }

    public double getRotatePowerCcwPositive() {
        double powerCwPos = driveController.getRightX();

        powerCwPos = Math.pow(powerCwPos, 3);

        return powerCwPos * -1;
    }

    public double getRightY() {
        return driveController.getRightY();
    }

    public boolean getFieldDrive() {
        return driveController.getBThreePosSwitch() != 1;
    }

    public boolean getWheelLock() {
        return driveController.getDButton();
    }

    public boolean resetGyro() {
        return false;
        // return driveController.getGButton();
    }

    public boolean getAutoAlign() {
        return false;
        // return driveController.getHButton();
    }

    public boolean getAutoAim() {
        return driveController.getFTwoPosSwitch();
    }

    public boolean enablePrecisionDrive() {
        return driveController.getETwoPosSwitch();
    }

    public boolean getClimberActuate() {
        return driveController.getAButton();
    }

    /*********************************************/
    /*                                           */
    /*              Xbox Controller              */
    /*                                           */
    /*********************************************/

    public boolean getPauseTurret() {
        return manipController.getAButton();
    }

    public boolean getShootButton() {
        return manipController.getRightTriggerAxis() > 0.8;
    }
    
    public boolean getReverseIndexer() {
        return manipController.getRightBumperButton();
    }

    public boolean getIntakeUp() {
        if (manipController.getPOV() == 315 || manipController.getPOV() == 45 || manipController.getPOV() == 0) {
            return true;
        }

        return false;
    }

    public boolean getIntakeDown() {
        if (manipController.getPOV() == 135 || manipController.getPOV() == 225 || manipController.getPOV() == 180) {
            return true;
        }

        return false;
    }

    public boolean getRunIntake() {
        return manipController.getLeftTriggerAxis() >= 0.8;
    }

    public boolean getReverseIntake() {
        return manipController.getLeftBumperButton();
    }

    public boolean getLeftAdjustButton() {
        return manipController.getXButton();
    }

    public boolean getRightAdjustButton() {
        return manipController.getYButton();
    }

    public boolean getLeftAdjustReleased() {
        if (!leftAdjusted && manipController.getXButtonReleased()) {
            leftAdjusted = true;
            return true;
        }

        return false;
    }

    public boolean getRightAdjustReleased() {
        if (!rightAdjusted && manipController.getYButtonReleased()) {
            rightAdjusted = true;
            return true;
        }

        return false;
    }

    public Translation2d getLeftAdjust() {
        return new Translation2d(manipController.getLeftX(), manipController.getLeftY());
    }

    public Translation2d getRightAdjust() {
        return new Translation2d(manipController.getRightX(), manipController.getRightY());
    }
}
