// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Songs;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

// TODO Parameterize this class so that a single implementation can play multiple files

public class bohemianRhapsody extends CommandBase {
  /** Creates a new bohemianRhapsody. */
  private Orchestra orchestra;
  public bohemianRhapsody(Swerve s_Swerve) {
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //driveSubsystem.stop();
    orchestra = new Orchestra();
    for (int i = 0; i < Constants.talonCount; i++){
      orchestra.addInstrument(new TalonFX(i));
    }
    orchestra.loadMusic("bohemianrhapsody.chrp");
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
