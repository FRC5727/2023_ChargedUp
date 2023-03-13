// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Position;

public class ArmCommand extends CommandBase {
  // Creates a new ArmCommand
  
  private final ArmSubsystem arm;
  private Position position = null;
  private Supplier<Position> positionSupplier = null;

  public ArmCommand(ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
  }

  public ArmCommand(ArmSubsystem arm, Position position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.position = position;
    addRequirements(arm);
  }

  public ArmCommand(ArmSubsystem arm, Supplier<Position> positionSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.positionSupplier = positionSupplier;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (positionSupplier != null) {
      position = positionSupplier.get();
    }
    if (position != null) {
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
