// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private final IntakeSubsystem intake;

  private boolean cube; //cone = 1, cube = 0
  

  public IntakeCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  public void coneIntake(){
    intake.setSpeed(-1.00);
  }
  public void cubeIntake(){
    intake.setSpeed(1);
  }
  public void coneOuttake(){
    intake.setSpeed(1);
  }
  public void cubeOuttake(){
    intake.setSpeed(-1);
  }
  public void place(){
    if(!cube) coneOuttake();
    if(cube) cubeOuttake();
  }
  public void intake(){
    if(!cube) coneIntake();
    if(cube) cubeIntake();
  }
  public void setCube(){
    cube = true;
  }
  public void setCone(){
    cube = false;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cube = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Constants.dXboxController.getLeftTriggerAxis() > 0.05) place();
    if(Constants.dXboxController.getRightTriggerAxis() > 0.05) intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
