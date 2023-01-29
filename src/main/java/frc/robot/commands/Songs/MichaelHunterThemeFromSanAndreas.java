// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Songs;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class MichaelHunterThemeFromSanAndreas extends CommandBase {
  /** Creates a new MichaelHunterThemeFromSanAndreas. */
  private Orchestra orchestra;
  private DriveSubsystem driveSubsystem;
  public MichaelHunterThemeFromSanAndreas(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //driveSubsystem.stop();
    orchestra = new Orchestra();
    for (int i = 0; i < Constants.talonCount; i++){
      orchestra.addInstrument(new TalonFX(i));
    }
    orchestra.loadMusic("MichaelHunterThemeFromSanAndreas.chrp");
    orchestra.play();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    orchestra.stop();
    orchestra = null;
    driveSubsystem.start();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
