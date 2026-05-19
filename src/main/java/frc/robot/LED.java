// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
public class LED {
    //Addressable Led 1 can be changed, it is the port #
    LED = new AddressableLED(1);
    LEDBuffer = new AddressableLEDBuffer(60);
    LED.setlength(LEDSBuffer.getlength());
    LEDPattern blue = LEDPattern.solid(Color.kBlue);
    blue.applyTo(LEDBuffer);
}
