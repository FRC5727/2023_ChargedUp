// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.highArmSubsystem;
import frc.robot.subsystems.lowerArmSubsystem;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  
  private final lowerArmSubsystem lowArm;
  private final highArmSubsystem highArm;

  public ArmCommand(lowerArmSubsystem lowArm, highArmSubsystem highArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lowArm = lowArm;
    this.highArm = highArm;
    addRequirements(lowArm, highArm);
  }

  public void chassisPos(){
    lowArm.setGoal(0);
    highArm.setGoal(0);
  }
  public void intakeGroundPos(){
    lowArm.setGoal(0);
    highArm.setGoal(0);
  }
  public void highPos(){
    lowArm.setGoal(0);
    highArm.setGoal(0);
  }
  public void midPos(){
    lowArm.setGoal(0);
    highArm.setGoal(0);
  }
  public void lowPos(){
    lowArm.setGoal(0);
    highArm.setGoal(0);
  }
  public void stationPickupPos(){
    lowArm.setGoal(0);
    highArm.setGoal(0);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.dXboxController.getPOV() == 0) midPos();
    if (Constants.dXboxController.getPOV() == 270) lowPos();
    if (Constants.dXboxController.getPOV() == 180) chassisPos();
    if (Constants.dXboxController.getPOV() == 90) highPos();
    if (Constants.dXboxController.getLeftBumperPressed()) intakeGroundPos();
    if (Constants.dXboxController.getRightBumperPressed()) stationPickupPos();
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
