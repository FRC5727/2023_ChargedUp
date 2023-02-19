// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

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
    intake.setSpeed(0.5);
  }

  public void cubeIntake(){
    intake.setSpeed(-0.5);
  }

  public void coneOuttake(){
    intake.setSpeed(-0.5);
  }
  public void cubeOuttake(){
    intake.setSpeed(0.5);
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

  public void cubeIdle(){
    intake.setSpeed(-.08);
  }
  public void coneIdle(){
    intake.setSpeed(0.08);
  }

  public void toggleCube(){
    cube = !cube;
  }

  public boolean isCube(){
    return cube;
  }
  public void intakeIdle(){
    if(!cube) coneIdle();
    if(cube) cubeIdle();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cube = true;
    System.out.println("TODO Start intake");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Constants.dXboxController.getLeftTriggerAxis() > 0.10) place();
    if(Constants.dXboxController.getRightTriggerAxis() > 0.10) intake();
    if(Constants.dXboxController.getLeftTriggerAxis() < 0.10 && Constants.dXboxController.getRightTriggerAxis() < 0.10) intakeIdle();
    System.out.println("TODO Intake running");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("TODO Stop intake");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO Check if piece acquired (motor stalled)
    return false;
  }
}
