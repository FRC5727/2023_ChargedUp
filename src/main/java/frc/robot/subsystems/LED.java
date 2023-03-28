package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  private CANdle m_candle = new CANdle(Constants.LED_CANDLE);
  private final int numLed = 114; // Includes numBaseLed
  private final int numBaseLed = 8;
  private final double middle = 52.5; // Not including numBaseLed
  private final double brightness = 0.50;
  private int flashCount = 0;
  private boolean flashOff = false;
  private final int flashRate = 8; // in ticks
  private int flashTicks = 0;
  private Color last = new Color(0, 0, 0);
  private Color flash = new Color(0, 0, 0);

  public static class Color {
    private final int R, G, B;
    public Color(int r, int g, int b) {
      R = r;
      G = g;
      B = b;
    }
  }

  public static final class Colors {
    public static final Color off = new LED.Color(0, 0, 0);
    public static final Color omegabytes = new LED.Color(162, 255, 0);
    public static final Color purple = new LED.Color(186, 0, 255);
    public static final Color yellow = new LED.Color(255, 128, 0);
    public static final Color disabledRed = new LED.Color(222, 0, 0);
    public static final Color blue = new LED.Color(0, 0, 222);
    public static final Color white = new LED.Color(255, 255, 255);
    public static final Color green = new LED.Color(0, 255, 0);
    public static final Color teal = new LED.Color(0, 255, 255);
  }

  public LED() {
    m_candle.configBrightnessScalar(brightness);
    m_candle.configLEDType(LEDStripType.GRB);
    m_candle.configV5Enabled(true);
    m_candle.configLOSBehavior(true);
    setColor(Colors.omegabytes);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (flashCount > 0) {
      flashTicks++;
      if (flashTicks == flashRate) {
        if (flashOff) {
          setColorDirect(last);
          flashOff = false;
          flashCount--;
        } else {
          setColorDirect(flash);
          flashOff = true;
        }
        flashTicks = 0;
      }
    }
  }

  private void setColorDirect(Color color) {
    setColorDirect(color, 0.0, 100.0);
  }

  private void setColorDirect(Color color, double startPct, double endPct) {
    // Turn off the base LEDs to prevent overheating
    m_candle.setLEDs(0, 0, 0, 0, 0, numBaseLed);

    double cnt = middle - 0.000001;
    int start = (int)Math.round(startPct / 100.0 * cnt) + numBaseLed;
    int end = (int)Math.round(endPct / 100.0 * cnt) + numBaseLed - 1;

    DriverStation.reportWarning("Setting front color between " + start + " and " + end, false);
    m_candle.setLEDs(color.R, color.G, color.B, 255, start, end - start + 1);

    cnt = numLed - numBaseLed - middle + 0.000001;
    start = numLed - (int)Math.round(endPct / 100.0 * cnt);
    end = numLed - 1 - (int)Math.round(startPct / 100.0 * cnt);

    DriverStation.reportWarning("Setting back color between " + start + " and " + end, false);
    m_candle.setLEDs(color.R, color.G, color.B, 255, start, end - start + 1);
  }

  public void setColor(Color color) {
    setColor(color, 0.0, 100.0);
  }

  public void setColor(Color color, double start, double end) {
    last = color;
    flashCount = 0;
    flashOff = false;
    m_candle.clearAnimation(0);
    setColorDirect(color, start, end);
  }

  public void setRainbow(){
    m_candle.clearAnimation(0);
    m_candle.animate(new RainbowAnimation(brightness, 0.5, numLed));
  }

  public void setLarson(Color color) {
    m_candle.clearAnimation(0);
    m_candle.animate(new LarsonAnimation(color.R, color.G, color.B, 255, 0.5, numLed, LarsonAnimation.BounceMode.Center, 3));
  }

  public void flash(int count) {
    flash(count, Colors.off);
  }

  public void flash(int count, Color color) {
    flashCount = count;
    flash = color;
  }
}