package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import frc.robot.Constants;

public class LED extends SubsystemBase {
  private CANdle m_candle = new CANdle(Constants.LED_CANDLE);
  private final int numLed = 114; // Includes numBaseLed
  private final int numBaseLed = 8;
  private final double middle = 52.5; // Not including numBaseLed
  private final double brightness = 0.50;
  private int strobeTicks = 0;
  private Color last = Colors.off;
  private int m_animations = 0;
  private int animExclude = 3;

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
    public static final Color red = new LED.Color(255, 0, 0);
    public static final Color green = new LED.Color(0, 255, 0);
    public static final Color blue = new LED.Color(0, 0, 255);
    public static final Color cyan = new LED.Color(0, 255, 255);
    public static final Color magenta = new LED.Color(255, 0, 255);
    public static final Color yellow = new LED.Color(255, 255, 0);
    public static final Color white = new LED.Color(255, 255, 255);
    public static final Color omegabytes = new LED.Color(162, 255, 0);
    public static final Color disabled = new LED.Color(200, 0, 0);
    public static final Color cube = new LED.Color(186, 0, 255);
    public static final Color cone = new LED.Color(255, 128, 0);
  }

  public LED() {
    m_candle.configBrightnessScalar(brightness);
    m_candle.configLEDType(LEDStripType.GRB);
    m_candle.configV5Enabled(true);
    m_candle.configLOSBehavior(true);
    setColor(Colors.omegabytes);

    // LED tests
    SmartDashboard.putData("LED Rainbow", Commands.startEnd(() -> setRainbow(), () -> setColor(Colors.omegabytes)));
    SmartDashboard.putData("LED Larson", Commands.startEnd(() -> setLarson(Colors.cyan), () -> setColor(Colors.omegabytes)));
    SmartDashboard.putData("LED Strobe", Commands.startEnd(() -> setStrobe(Colors.cyan), () -> setColor(Colors.omegabytes)));
    SmartDashboard.putData("LED Flash", Commands.startEnd(() -> flash(50, Colors.cyan), () -> setColor(Colors.omegabytes)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (strobeTicks-- > 0) {
      if (strobeTicks == 0) {
        clearAnimations();
        setColorDirect(last);
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

    m_candle.setLEDs(color.R, color.G, color.B, 255, start, end - start + 1);

    cnt = numLed - numBaseLed - middle + 0.000001;
    start = numLed - (int)Math.round(endPct / 100.0 * cnt);
    end = numLed - 1 - (int)Math.round(startPct / 100.0 * cnt);

    m_candle.setLEDs(color.R, color.G, color.B, 255, start, end - start + 1);
  }

  public void setColor(Color color) {
    setColor(color, 0.0, 100.0);
  }

  public void clearAnimations() {
    for (int idx = 0; idx <= m_animations; idx++) {
      m_candle.clearAnimation(idx);
    }
    m_animations = 0;
  }

  public void setColor(Color color, double start, double end) {
    last = color;
    strobeTicks = 0;
    clearAnimations();
    setColorDirect(color, start, end);
  }

  public void setRainbow() {
    clearAnimations();
    m_candle.animate(new RainbowAnimation(brightness, 0.5, numLed, false, numBaseLed));
    m_animations = 1;
  }

  public void setLarson(Color color) {
    clearAnimations();
    m_candle.animate(new LarsonAnimation(color.R, color.G, color.B, 255, 0.8, (int)Math.round(middle - animExclude - 0.000001), LarsonAnimation.BounceMode.Front, 7, numBaseLed), 0);
    m_candle.animate(new LarsonAnimation(color.R, color.G, color.B, 255, 0.8, (int)Math.round(numLed - numBaseLed - middle - animExclude + 0.000001), LarsonAnimation.BounceMode.Front, 7, (int)Math.round(middle + numBaseLed + animExclude)), 1);
    m_animations = 2;
  }

  public void setStrobe(Color color) {
    clearAnimations();
    m_candle.animate(new StrobeAnimation(color.R, color.G, color.B, 255, 0.1, (numLed - numBaseLed) / 2 - animExclude, numBaseLed), 0);
    m_candle.animate(new StrobeAnimation(color.R, color.G, color.B, 255, 0.1, (numLed - numBaseLed) / 2 - animExclude, (numLed - numBaseLed) / 2 + animExclude + numBaseLed), 1);
    m_animations = 2;
  }

  public void setStrobe(Color color, int ticks) {
    this.strobeTicks = ticks;
    setStrobe(color);
  }

  public void flash(int count) {
    flash(count, Colors.off);
  }

  public void flash(int count, Color color) {
    setStrobe(color, count);
  }
}