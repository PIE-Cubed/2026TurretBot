package frc.robot;

import java.util.Map;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LED {
    // you forgot to add the specifiers here
    // private/public VariableType variableName = new VariableType();
    // we also typically use final variables at the top of the class for things that get changed manually, like the LED port here
    private final int LED_PORT = 1;
    private AddressableLED LED = new AddressableLED(LED_PORT);
    private AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(10);

    // moved the pattern you had here so it can be stored for easier access
    // i would probably try to put most patterns up here though you don't have to
    // private final LEDPattern disabledPattern = LEDPattern.solid(Color.kBlue); //One color
    //private final LEDPattern steps = LEDPattern.steps(Map.of(0, Color.kYellow, .25, Color.kBlue, .75, Color.kYellow)); //Different section of lights
    //private final AddressableLEDBufferView   leftHopper = LEDBuffer.createView(0, 15); //Left section, change # to change amount of lights in section
    //private final AddressableLEDBufferView middleHopper = LEDBuffer.createView(16,57); //Middle section
    //private final AddressableLEDBufferView  rightHopper = LEDBuffer.createView(58,72); //Right section

    private final LEDPattern solidYellow = LEDPattern.solid(Color.kYellow);
    private final LEDPattern solidBlue   = LEDPattern.solid(Color.kBlue);

    
    // the other code you had was outside of a constructor or other function so it didn't like that
    // Taco belly
    public LED() {
        LED.setLength(LEDBuffer.getLength());
        LED.setData(LEDBuffer);
        LED.start();
        
        // solidYellow.applyTo(leftHopper);
        // solidBlue.applyTo(middleHopper);
        // solidYellow.applyTo(rightHopper);

        //steps.applyTo(LEDBuffer);
        //disabledPattern.applyTo(LEDBuffer);
    }

    public void periodic() {
        solidYellow.applyTo(LEDBuffer);
        LED.setData(LEDBuffer);
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
