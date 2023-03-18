package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Dashboard;

public class IntakeSubsystem extends SubsystemBase {
  private boolean intakeDebug = false;
  private final CANSparkMax intakeNeo;
  private final LED m_led;

  private boolean cube = true;
  private boolean colorInit = false;
  private double currentSpeed = 0.0;
  private int stallCounter = 0;

  public IntakeSubsystem(LED s_LED) {
    intakeNeo = new CANSparkMax(Constants.Intake.deviceId, MotorType.kBrushless);
    
    Dashboard.watchBoolean("Intake debug", intakeDebug, (val) -> intakeDebug = val.booleanValue());

    m_led = s_LED;
  }

  public void setSpeed(double speed) {
    currentSpeed = speed;
    intakeNeo.set(speed);
    if (!colorInit) {
      setColor();
    }
  }

  public void coneIntake(){
    setSpeed(Constants.Intake.intakeSpeed);
  }

  public void cubeIntake(){
    setSpeed(-1.0 * Constants.Intake.intakeSpeed);
  }

  public void coneOuttake(){
    setSpeed(-1.0 * Constants.Intake.outtakeConeSpeed);;
  }

  public void cubeOuttake(){
    setSpeed(Constants.Intake.outtakeSpeed);
  }

  public void place(){
    if(isCube()) {
      cubeOuttake();
    } else {
      coneOuttake();
    }
  }

  public void intake(){
    if(isCube()) {
      cubeIntake();
    } else {
      coneIntake();
    }
  }
  
  public void cubeIdle(){
    setSpeed(-1.0 * Constants.Intake.idleSpeed);
  }
  
  public void coneIdle(){
    setSpeed(Constants.Intake.idleSpeed);
  }
  
  public void toggleCube(){
    cube = !cube;
    setSpeed(-1.0 * currentSpeed);
    setColor();
  }
  
  private void setColor() {
    if (cube) {
      m_led.setColor(LED.Colors.purple);
    } else {
      m_led.setColor(LED.Colors.yellow);
    }
    colorInit = true;
  }
  
  public void disabled() {
    if (colorInit) {
      m_led.setColor(LED.Colors.disabledRed);
      colorInit = false;
    }
  }

  public void flash() {
    m_led.flash(5, LED.Colors.omegabytes);
  }
  
  public boolean isCube(){
    return cube;
  }
  
  public boolean isStalled(){
    return stallCounter >= (cube ? Constants.Intake.stallMaxCube : Constants.Intake.stallMaxCone);
  }
  
  public void intakeIdle(){
    if(isCube()) {
      cubeIdle();
    } else {
      coneIdle();
    }
  }
  
  @Override
  public void periodic() {
    if (intakeNeo.getOutputCurrent() > Constants.Intake.stallCurrent) {
      stallCounter++;
    } else {
      stallCounter = 0;
    }
    SmartDashboard.putBoolean("Cube?", cube);
    if (intakeDebug) {
      SmartDashboard.putNumber("Intake output current", intakeNeo.getOutputCurrent());
      SmartDashboard.putNumber("Intake stall counter", stallCounter);
      SmartDashboard.putBoolean("Piece captured?", isStalled());
    }
  }
}