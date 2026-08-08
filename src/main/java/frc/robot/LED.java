package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.AllianceUtil;

public class LED {
    // you forgot to add the specifiers here
    // private/public VariableType variableName = new VariableType();
    // we also typically use final variables at the top of the class for things that get changed manually, like the LED port here
    private final int LED_PORT = 1;

    private AddressableLED LED = new AddressableLED(LED_PORT);
    private AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(71);

    private final AddressableLEDBufferView  rightHopper = LEDBuffer.createView(0, 13); //Left section, change # to change amount of lights in section
    private final AddressableLEDBufferView middleHopper = LEDBuffer.createView(14,55); //Middle section
    private final AddressableLEDBufferView  leftHopper = LEDBuffer.createView(56,70); //Right section

    private final LEDPattern normalTeamHub = LEDPattern.steps(Map.of(0, Color.kBlue, 0.2113, Color.kYellow, 0.7888, Color.kBlue));

    // moved the pattern you had here so it can be stored for easier access
    // i would probably try to put most patterns up here though you don't have to
    // private final LEDPattern disabledPattern = LEDPattern.solid(Color.kBlue); // One color
    
    private final LEDPattern solidYellow = LEDPattern.solid(Color.kYellow);
    private final LEDPattern solidBlue   = LEDPattern.solid(Color.kBlue);
    private final LEDPattern solidOrange = LEDPattern.solid(new Color(248, 27, 0));
    private final LEDPattern off         = LEDPattern.solid(Color.kBlack);

    private final LEDPattern patriotic = LEDPattern.steps(Map.of(0, Color.kRed, 0.33, Color.kWhite, 0.67, Color.kBlue));
    private final LEDPattern scrollingPatriotic = patriotic.scrollAtRelativeSpeed(Percent.per(Second).of(20.0)).atBrightness(Percent.of(200));

    private final LEDPattern blinkRSL = solidOrange.synchronizedBlink(RobotController::getRSLState);

    private final LEDPattern blinkHub = normalTeamHub.blink(Second.of(0.5));

    // the other code you had was outside of a constructor or other function so it didn't like that
    // Taco belly
    public LED() {
        LED.setLength(LEDBuffer.getLength());
        LEDPattern.kOff.applyTo(LEDBuffer);
        LED.setData(LEDBuffer);
        LED.start();
    }

    public void periodic() {
        LED.setData(LEDBuffer);
    }

    public void applyTeamColors() {
        solidBlue.applyTo(leftHopper);
        solidYellow.applyTo(middleHopper);
        solidBlue.applyTo(rightHopper);
        //normalTeamHub.applyTo(LEDBuffer);
    }

    public void applyPatrioticColors() {
        scrollingPatriotic.applyTo(LEDBuffer);
    }

    public void applyRSLSync() {
        blinkRSL.applyTo(LEDBuffer);
    }
     
    public void applyHubSync() {
        double currTime = DriverStation.getMatchTime();

        // use else {} blocks WAY more often joey
        
        // currently in autonomous
        if (DriverStation.isAutonomous()) {
            solidBlue.applyTo(leftHopper);
            solidYellow.applyTo(middleHopper);
            solidBlue.applyTo(rightHopper);
        } // currently in endgame
        else if (AllianceUtil.isEndgame(currTime)) {
            solidBlue.applyTo(leftHopper);
            solidYellow.applyTo(middleHopper);
            solidBlue.applyTo(rightHopper);
        } // our hub is active in teleop
        else if (AllianceUtil.isOurHubActive(currTime) == true) {
            solidBlue.applyTo(leftHopper);
            solidYellow.applyTo(middleHopper);
            solidBlue.applyTo(rightHopper);

            // the hub is about to turn off
            if (AllianceUtil.timeUntilHubStateChange(currTime) <= 5) {
                blinkHub.applyTo(LEDBuffer);
            }
        } // our hub is not active in teleop
        else {
            off.applyTo(LEDBuffer);

            // the hub is about to turn on
            if (AllianceUtil.timeUntilHubStateChange(currTime) <= 5) {
                blinkHub.applyTo(LEDBuffer);
            }
        }
    }
        

    /*
     * TODO: Add other fancy LED functions
     * try to do these things with the LEDs
     * 
     *  - make seperate buffers of lights for seperate sections of the light strip, 
     *    so for example frontHopperLEDBuffer, leftHopperLEDBuffer, rightTurretLEDBuffer, etc (Good I hope)
     * 
     *  - during the off shift, make the LEDs pulse faster leading up to the on shift
     *    current shift data can be accessed from AllianceUtil
     * 
     *  - make a pattern that flashes orange along with the RSL
     * 
     *  - make a pattern that does a rotating green UFO-like thing
     *    the function to set this should have an input variable for how fast it should spin
     * 
     *  - add some way to set the pattern
     *    however you do it make the functions public (NOT static or private) (I think I did this just not public)
     * 
     *  - do whatever else you can think of (Send Morse Code that says "it is build's fault")
     */
}
