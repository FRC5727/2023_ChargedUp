// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerDistributionSubsystem extends SubsystemBase {
  /** Creates a new PowerDistribution. */
  PowerDistribution powerDistribution;
  int 
  slot1, 
  slot2, 
  slot3, 
  slot4, 
  slot5, 
  slot6, 
  slot7, 
  slot8, 
  slot9, 
  slot10, 
  slot11, 
  slot12, 
  slot13, 
  slot14, 
  slot15,
  slot16,
  slot17,
  slot18,
  slot19,
  slot20,
  slot21,
  slot22,
  slot23,
  slot24;
  public PowerDistributionSubsystem() {
    powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
