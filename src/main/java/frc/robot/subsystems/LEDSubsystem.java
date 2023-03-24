// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  /** Creates a new LEDSubsystem. */
  private CANdle m_candle = new CANdle(Constants.LED_CANDLE);
  private final int numLed = 114;
  private final double brightness = 0.50;
  private int flashCount = 0;
  private boolean flashOff = false;
  private final int flashRate = 8; // in ticks
  private int flashTicks = 0;
  private int lastR = 0;
  private int lastG = 0;
  private int lastB = 0;
  private int flashR = 0;
  private int flashG = 0;
  private int flashB = 0;

  public LEDSubsystem() {
    m_candle.configBrightnessScalar(brightness);
    m_candle.configLEDType(LEDStripType.GRB);
    m_candle.configV5Enabled(true);
    m_candle.configLOSBehavior(true);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (flashCount > 0) {
      flashTicks++;
      if (flashTicks == flashRate) {
        if (flashOff) {
          m_candle.setLEDs(lastR, lastG, lastB, 255, 0, numLed);
          flashOff = false;
          flashCount--;
        } else {
          m_candle.setLEDs(flashR, flashG, flashB, 255, 0, numLed);
          flashOff = true;
        }
        flashTicks = 0;
      }
    }
  }
  public void setColor(int r, int g, int b) {
    lastR = r;
    lastG = g;
    lastB = b;
    flashCount = 0;
    flashOff = false;
    m_candle.clearAnimation(0);
    m_candle.setLEDs(r, g, b, 255, 0, numLed);
  }
  public void setRainbow(){
    m_candle.clearAnimation(0);
    m_candle.animate(new RainbowAnimation(brightness, 4, numLed));
  }
  public void flash(int count) {
    flash(count, 0, 0, 0);
  }
  public void flash(int count, int r, int g, int b) {
    flashCount = count;
    flashR = r;
    flashG = g;
    flashB = b;
  }
}
