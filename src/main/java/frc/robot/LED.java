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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Optional;
import frc.robot.util.AllianceUtil;

public class LED {
    // you forgot to add the specifiers here
    // private/public VariableType variableName = new VariableType();
    // we also typically use final variables at the top of the class for things that get changed manually, like the LED port here
    private final int LED_PORT = 1;

    private AddressableLED LED = new AddressableLED(LED_PORT);
    private AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(71);

    private final AddressableLEDBufferView  rightHopper = LEDBuffer.createView(0, 14); //Left section, change # to change amount of lights in section
    private final AddressableLEDBufferView middleHopper = LEDBuffer.createView(15,54); //Middle section
    private final AddressableLEDBufferView  leftHopper = LEDBuffer.createView(55,70); //Right section

    private final LEDPattern normalTeamHub = LEDPattern.steps(Map.of(0, Color.kBlue, .22, Color.kYellow, .8, Color.kBlue ));

    // moved the pattern you had here so it can be stored for easier access
    // i would probably try to put most patterns up here though you don't have to
    // private final LEDPattern disabledPattern = LEDPattern.solid(Color.kBlue); //One color
    // private final LEDPattern steps = LEDPattern.steps(Map.of(0, Color.kYellow, .25, Color.kBlue, .75, Color.kYellow)); //Different section of lights
    
    private final LEDPattern solidYellow = LEDPattern.solid(Color.kYellow);
    private final LEDPattern solidBlue   = LEDPattern.solid(Color.kBlue);
    private final LEDPattern solidOrange = LEDPattern.solid(new Color(248, 27, 0));
    private final LEDPattern off         = LEDPattern.solid(Color.kBlack);

    private final LEDPattern patriotic = LEDPattern.steps(Map.of(0, Color.kRed, 0.33, Color.kWhite, 0.67, Color.kBlue));
    private final LEDPattern scrollingPatriotic = patriotic.scrollAtRelativeSpeed(Percent.per(Second).of(20.0)).atBrightness(Percent.of(200));

    private final LEDPattern blinkRSL = solidOrange.synchronizedBlink(RobotController :: getRSLState);

    private final LEDPattern blinkHub = normalTeamHub.blink(Second.of(0.5));


    private AllianceUtil allianceUtil;
    

    // the other code you had was outside of a constructor or other function so it didn't like that
    // Taco belly
    public LED() {
        LED.setLength(LEDBuffer.getLength());
        LEDPattern.kOff.applyTo(LEDBuffer);
        allianceUtil = new AllianceUtil();
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
     
    /*  public void applyHubSync() {
        
         if (DriverStation.isAutonomous()) {
            solidBlue.applyTo(leftHopper);
            solidYellow.applyTo(middleHopper);
            solidBlue.applyTo(rightHopper);
        }
        
        if ((allianceUtil.isOurHubActive(DriverStation.getMatchTime()) == true) && 
            (allianceUtil.timeUntilHubStateChange(DriverStation.getMatchTime()) <= 5)) {
            blinkHub.applyTo(LEDBuffer);
        }

        if (allianceUtil.isOurHubActive(DriverStation.getMatchTime()) == true) {
            solidBlue.applyTo(leftHopper);
            solidYellow.applyTo(middleHopper);
            solidBlue.applyTo(rightHopper);
        }

        if (allianceUtil.isOurHubActive(DriverStation.getMatchTime()) == false) {
            off.applyTo(LEDBuffer);
        }

        if ((allianceUtil.isOurHubActive(DriverStation.getMatchTime()) == false) && 
            (allianceUtil.timeUntilHubStateChange(DriverStation.getMatchTime()) <= 5)) {
            blinkHub.applyTo(LEDBuffer);
        }
        */
       /* if (allianceUtil.timeUntilHubStateChange(6.0) <= 5) {
        blinkHub.applyTo(LEDBuffer);
        }
        */
        /* if (allianceUtil.isEndgame(ENDGAME_SECONDS)) {
            solidBlue.applyTo(leftHopper);
            solidYellow.applyTo(middleHopper);
            solidBlue.applyTo(rightHopper);
        }
        
        
    }*/
        

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
