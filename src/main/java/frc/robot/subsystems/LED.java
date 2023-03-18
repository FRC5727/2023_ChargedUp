package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  private CANdle m_candle = new CANdle(Constants.LED_CANDLE);
  private final int numLed = 114;
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
    m_candle.setLEDs(color.R, color.G, color.B, 255, 0, numLed);
  }

  public void setColor(Color color) {
    last = color;
    flashCount = 0;
    flashOff = false;
    m_candle.clearAnimation(0);
    setColorDirect(color);
  }

  public void setRainbow(){
    m_candle.clearAnimation(0);
    m_candle.animate(new RainbowAnimation(brightness, 0.3, numLed));
  }

  public void flash(int count) {
    flash(count, Colors.off);
  }

  public void flash(int count, Color color) {
    flashCount = count;
    flash = color;
  }
}