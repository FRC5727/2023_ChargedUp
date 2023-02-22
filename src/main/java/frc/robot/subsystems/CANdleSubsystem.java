// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSubsystem extends SubsystemBase {
  /** Creates a new CANdleSubsystem. */
  // private CANdle brightBoi = new CANdle(0, getName());

  public CANdleSubsystem() {
    // CANdleConfiguration config = new CANdleConfiguration();
    // config.stripType = LEDStripType.RGB; // set the strip type to RGB
    // config.brightnessScalar = 1.0; // LED Brightness Scalar
    // brightBoi.configAllSettings(config);

  }
  // public void idleLED(){
  //   brightBoi.setLEDs(162, 255, 0);
  // }
  // public void coneLED(){
  //   brightBoi.setLEDs(255, 255, 0);
  // }
  // public void cubeLED(){
  //   brightBoi.setLEDs(162, 0, 128);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // create a rainbow animation:

    // - max brightness

    // - half speed

    // - 64 LEDs

    // RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 64);

    // brightBoi.animate(rainbowAnim);
  }
}
