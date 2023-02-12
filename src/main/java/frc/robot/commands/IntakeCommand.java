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

  private int cubeOrCone; //cone = 1, cube = 0
  

  public IntakeCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  public void coneIntake(){
    intake.setSpeed(0, 0);
  }
  public void cubeIntake(){
    intake.setSpeed(0, 0);
  }
  public void coneOuttake(){
    intake.setSpeed(0, 0);
  }
  public void cubeOuttake(){
    intake.setSpeed(0, 0);
  }
  public void place(){
    if(cubeOrCone == 1) coneOuttake();
    if(cubeOrCone == 0) cubeOuttake();
  }
  public void intake(){
    if(cubeOrCone == 1) coneIntake();
    if(cubeOrCone == 0) cubeIntake();
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Constants.dXboxController.getRawButtonReleased(1)) cubeOrCone = 0;
    if(Constants.dXboxController.getRawButtonReleased(2)) cubeOrCone = 1; //b 
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
