// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Position;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  
  private final ArmSubsystem arm;
  private final ArmSubsystem.Position position = Position.CHASSIS;
  private final boolean positionSet = false;

  public ArmCommand(ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
  }

  public ArmCommand(ArmSubsystem arm, ArmSubsystem.Position position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.position = position;
    positionSet = true;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (positionSet) {
      arm.setTargetPosition(position);
    }
    arm.beginMovement();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.updateMovement();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopMovement();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.doneMovement();
  }
}
