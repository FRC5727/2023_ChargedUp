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
      if (cube) {
        setSpeed(Constants.Intake.idleCubeSpeed * -1.0);
      } else {
        setSpeed(Constants.Intake.idleConeSpeed);
      }
    } else if (intake) {
      if (cube) {
        setSpeed(Constants.Intake.intakeCubeSpeed * -1.0);
      } else {
        setSpeed(Constants.Intake.intakeConeSpeed);
      }
    } else if (cube) {
      setSpeed(Constants.Intake.outtakeCubeSpeed);
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
    m_led.setLarson(cube ? LED.Colors.cube : LED.Colors.cone);
  }

  public void intake() {
    idle = false;
    intake = true;
    setSpeed();
    m_led.setLarson(cube ? LED.Colors.cube : LED.Colors.cone);
  }
  
  public void toggleCube() {
    cube = !cube;
    setSpeed();
    setColor();
  }
  
  public void setColor() {
    if (!isStalled()) {
      if (cube) {
        m_led.setColor(LED.Colors.cube);
      } else {
        m_led.setColor(LED.Colors.cone);
      }
      colorInit = true;
    }
  }
  
  public void disabled() {
    if (colorInit) {
      m_led.setColor(LED.Colors.disabled);
      colorInit = false;
    }
  }

  public void flash() {
    m_led.flash(50, LED.Colors.cyan);
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
    setColor();
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
      SmartDashboard.putNumber("Intake current speed", intakeNeo.get());
      SmartDashboard.putNumber("Intake stall counter", stallCounter);
      SmartDashboard.putBoolean("Piece captured?", isStalled());
    }
  }
}