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

  public LEDSubsystem() {
    m_candle.configBrightnessScalar(brightness);
    m_candle.configLEDType(LEDStripType.GRB);
    m_candle.configV5Enabled(true);
    // m_candle.configLOSBehavior(true);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setColor(int r, int g, int b){
    m_candle.clearAnimation(0);
    m_candle.setLEDs(r, g, b, 255, 0, numLed);
  }
  public void setRainbow(){
    m_candle.clearAnimation(0);
    m_candle.animate(new RainbowAnimation(brightness, 0.3, numLed));
  }
}
