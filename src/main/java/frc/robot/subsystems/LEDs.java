// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private AddressableLED m_led = new AddressableLED(9);
  private Boolean ambulanceMode = false;
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(64);
  private Color[] ambulanceList = {Color.kRed, Color.kBlue, Color.kWhite};
  /** Creates a new LEDs. */
  public LEDs() {
    m_led.setLength(64);
  }

  @Override
  public void periodic() {
    if (ambulanceMode) {
      setLEDS(ambulanceList[((int)(Math.random() * 2))]);
    }
    // This method will be called once per scheduler run
  }

  public void toggleAmbulance() {
    ambulanceMode = !ambulanceMode;
  }

  public void setLEDS(Color color) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setLED(i, color);
   }
   m_led.setData(m_ledBuffer);
   m_led.start();
  }

  public void LEDsOff() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, Color.kBlack);
    }
    m_led.setData(m_ledBuffer);
  }

}
