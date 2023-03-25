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
  private boolean idle = true;
  private boolean intake = false;
  private boolean colorInit = false;
  private int stallCounter = 0;

  public IntakeSubsystem(LED s_LED) {
    intakeNeo = new CANSparkMax(Constants.Intake.deviceId, MotorType.kBrushless);
    
    Dashboard.watchBoolean("Intake debug", intakeDebug, (val) -> intakeDebug = val.booleanValue());

    m_led = s_LED;
  }

  public void setSpeed() {
    if (idle) {
      setSpeed(Constants.Intake.idleSpeed * (cube ? -1.0 : 1.0));
    } else if (intake) {
      setSpeed(Constants.Intake.intakeSpeed * (cube ? -1.0 : 1.0));
    } else if (cube) {
      setSpeed(Constants.Intake.outtakeSpeed);
    } else {
      setSpeed(Constants.Intake.outtakeConeSpeed * -1.0);
    }
  }

  public void setSpeed(double speed) {
    intakeNeo.set(speed);
    if (!colorInit) {
      setColor();
    }
  }

  public void place() {
    idle = false;
    intake = false;
    setSpeed();
  }

  public void intake() {
    idle = false;
    intake = true;
    setSpeed();
  }
  
  public void toggleCube() {
    cube = !cube;
    setSpeed();
    setColor();
  }
  
  public void setColor() {
    // TODO Animate while intaking / placing
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
  
  public boolean isCube() {
    return cube;
  }
  
  public boolean isStalled() {
    return stallCounter >= (cube ? Constants.Intake.stallMaxCube : Constants.Intake.stallMaxCone);
  }
  
  public void idle() {
    idle = true;
    intake = true;
    setSpeed();
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