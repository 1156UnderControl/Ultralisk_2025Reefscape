package frc.robot.subsystems.leds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.LEDs.Color;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.robot.constants.ElevatorConstants;

class LedMode {
  public static final String SOLID = "SOLID";
  public static final String RAINBOW = "RAINBOW";
  public static final String BLINK = "BLINK";
  public static final String PERHEIGHT = "PERHEIGHT";
  public static final String SEGMENTED = "SEGMENTED";
  public static final String HEIGHT_SEGMENTED = "HEIGHT_SEGMENTED";
}

public class LedSubsystem extends SubsystemBase implements ILed {
  private static LedSubsystem instance = null;

  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private int rainbowFirstPixelHue;

  private final int intakeLeftStart = 0;
  private final int intakeLeftEnd = 14;
  private final int intakeRightStart = 15;
  private final int intakeRightEnd = 29;

  private final int elevatorLeftStart = 30;
  private final int elevatorLeftEnd = 52;
  private final int elevatorRightStart = 53;
  private final int elevatorRightEnd = 77;

  private final int intakeStart = 0;
  private final int intakeEnd = 29;
  private final int elevatorStart = 30;
  private final int elevatorEnd = 77;

  private Color intakeColor = LedColor.OFF;
  private Color elevatorColor = LedColor.OFF;
  private Color intakeLeftHeightColor = LedColor.OFF;
  private Color intakeRightHeightColor = LedColor.OFF;
  private Color elevatorLeftHeightColor = LedColor.OFF;
  private Color elevatorRightHeightColor = LedColor.OFF;

  private Timer blinkTimer = new Timer();

  private double blinkFrequency;

  private double blinkPeriod;

  private double halfBlinkPeriod;

  private int numberOfBlinks;

  private Color colorAfterBlink;

  private Color segmentColor;

  private String mode;

  private Color color;

  Supplier<Double> elevatorPosition;

  private double elevatorMinHeight = ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT;

  private double elevatorMaxHeight = ElevatorConstants.tunning_values_elevator.setpoints.MAX_HEIGHT;

  private CustomStringLogger modeLogEntry = new CustomStringLogger("/LedSubsystem/Mode");

  private CustomStringLogger colorLogEntry = new CustomStringLogger("/LedSubsystem/Color");

  private CustomBooleanLogger blinkTrackerLogEntry = new CustomBooleanLogger("/LedSubsystem/BlinkTracker");

  public static LedSubsystem getInstance(Supplier<Double> elevatorPosition) {
    if (instance == null) {
      instance = new LedSubsystem(elevatorPosition);
    }
    return instance;
  }

  private LedSubsystem(Supplier<Double> elevatorPosition) {
    this.elevatorPosition = elevatorPosition;
    this.setBlinkFrequency(3);
    this.mode = LedMode.SOLID;
    this.color = LedColor.OFF;
    this.led = new AddressableLED(8);
    this.ledBuffer = new AddressableLEDBuffer(78);
    this.led.setLength(ledBuffer.getLength());
    this.led.setData(ledBuffer);
    this.led.start();
    this.blinkTrackerLogEntry.append(false);
    this.periodic();
  }

  @Override
  public void periodic() {
    switch (this.mode) {
      case LedMode.SOLID:
        this.processSolidColor(this.color);
        break;
      case LedMode.SEGMENTED:
        this.processSegments();
        break;
      case LedMode.RAINBOW:
        this.processRainbow();
        break;
      case LedMode.BLINK:
        processBlink();
        break;
      case LedMode.PERHEIGHT:
        processHeight();
        break;
      case LedMode.HEIGHT_SEGMENTED:
        processHeightAnimation();
        break;
    }
    this.modeLogEntry.append(this.mode);
    this.colorLogEntry.append(this.color.name);
  }

  public void setBlinkFrequency(double frequency) {
    this.blinkFrequency = frequency;
    this.blinkPeriod = 1 / this.blinkFrequency;
    this.halfBlinkPeriod = this.blinkPeriod / 2;
  }

  public void turnOff() {
    this.setSolidColor(LedColor.OFF);
  }

  public void setSolidColor(Color color) {
    this.mode = LedMode.SOLID;
    this.color = color;
    this.blinkTrackerLogEntry.append(false);
    this.processSolidColor(this.color);
  }

  public void setBlink(Color color, int numberOfBlinks, Color colorAfterBlink) {
    this.mode = LedMode.BLINK;
    this.color = color;
    this.numberOfBlinks = numberOfBlinks;
    this.colorAfterBlink = colorAfterBlink;
    this.blinkTimer.reset();
    this.blinkTimer.start();
    this.processBlink();
  }

  private void setColorRange(int start, int end, Color color) {
    for (int i = start; i <= end; i++) {
      ledBuffer.setRGB(i, color.red, color.blue, color.green);
    }
  }

  public void setElevatorColor(Color color) {
    this.elevatorColor = color;
    this.mode = LedMode.SEGMENTED;
    this.processSegments();
  }

  public void setIntakeColor(Color color) {
    this.intakeColor = color;
    this.mode = LedMode.SEGMENTED;
    this.processSegments();
  }

  public void setIntakeHeightColor(Color color) {
    this.elevatorRightHeightColor = color;
    this.mode = LedMode.HEIGHT_SEGMENTED;
    this.processHeightAnimation();
  }

  public void setElevatorHeightColor(Color color) {
    this.elevatorRightHeightColor = color;
    this.mode = LedMode.HEIGHT_SEGMENTED;
    this.processHeightAnimation();
  }

  public void setHeightColor(Color color) {
    this.elevatorRightHeightColor = color;
    this.mode = LedMode.HEIGHT_SEGMENTED;
    this.processHeightAnimation();
  }

  public void setElevatorRightHeightColor(Color color) {
    this.elevatorRightHeightColor = color;
    this.mode = LedMode.HEIGHT_SEGMENTED;
    this.processHeightAnimation();
  }

  public void setElevatorLeftHeightColor(Color color) {
    this.elevatorLeftHeightColor = color;
    this.mode = LedMode.HEIGHT_SEGMENTED;
    this.processHeightAnimation();
  }

  public void setIntakeLeftHeightColor(Color color) {
    this.mode = LedMode.HEIGHT_SEGMENTED;
    this.intakeLeftHeightColor = color;
    this.processHeightAnimation();
  }

  public void setIntakeRightHeightColor(Color color) {
    this.mode = LedMode.HEIGHT_SEGMENTED;
    this.intakeRightHeightColor = color;
    this.processHeightAnimation();
  }

  public void setElevatorHeight(double height, double maxHeight) {
    this.mode = LedMode.PERHEIGHT;
    this.color = LedColor.RED;
    this.elevatorMinHeight = elevatorMinHeight;
    this.elevatorMaxHeight = elevatorMaxHeight;
  }

  public void setBlink(Color color, int numberOfBlinks) {
    this.setBlink(color, numberOfBlinks, LedColor.OFF);
  }

  public void setBlink(Color color) {
    this.setBlink(color, 0, LedColor.OFF);
  }

  public void setRainbow() {
    this.mode = LedMode.RAINBOW;
    this.color = LedColor.OFF;
    this.blinkTrackerLogEntry.append(false);
    this.processRainbow();
  }

  private void processRainbow() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var hue = (this.rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Define o valor do pixel
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    this.rainbowFirstPixelHue += 3;
    // Verifica Limites
    this.rainbowFirstPixelHue %= 180;
    led.setData(ledBuffer);
  }

  private void processHeight() {
    int ledStart = this.elevatorStart;
    int ledEnd = this.elevatorEnd;
    int ledRange = ledEnd - ledStart + 1;

    double currentHeight = elevatorPosition.get();
    double normalizedHeight = Math.min(1.0,
        Math.max(0.0, (currentHeight - elevatorMinHeight) / (elevatorMaxHeight - elevatorMinHeight)));

    int ledsToLight = (int) Math.round(normalizedHeight * ledRange);

    for (int i = 0; i < ledRange; i++) {
      int ledIndex = ledStart + i;
      if (i < ledsToLight) {
        ledBuffer.setRGB(ledIndex, color.red, color.blue, color.green);
      } else {
        ledBuffer.setRGB(ledIndex, 0, 0, 0);
      }
    }

    led.setData(ledBuffer);
  }

  private void processSegments() {
    for (int i = intakeStart; i <= intakeEnd; i++) {
      ledBuffer.setRGB(i, intakeColor.red, intakeColor.blue, intakeColor.green);
    }

    for (int i = elevatorStart; i <= elevatorEnd; i++) {
      ledBuffer.setRGB(i, elevatorColor.red, elevatorColor.blue, elevatorColor.green);
    }

    led.setData(ledBuffer);
  }

  private void processSolidColor(Color solidColor) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, solidColor.red, solidColor.blue, solidColor.green);
    }
    led.setData(ledBuffer);
  }

  private void processBlink() {
    double elapsedTime = this.blinkTimer.get();
    if (this.numberOfBlinks > 0) {
      double elapsedBlinkPeriods = elapsedTime / this.blinkPeriod;
      if (elapsedBlinkPeriods > this.numberOfBlinks) {
        this.setSolidColor(this.colorAfterBlink);
        return;
      }

    }
    if (elapsedTime % this.blinkPeriod < this.halfBlinkPeriod) {
      this.processSolidColor(this.color);
      this.blinkTrackerLogEntry.append(true);
    } else {
      this.processSolidColor(LedColor.OFF);
      this.blinkTrackerLogEntry.append(false);
    }
  }

  private void processHeightAnimation() {
    double currentHeight = elevatorPosition.get();
    double normalizedHeight = Math.min(1.0,
        Math.max(0.0, (currentHeight - elevatorMinHeight) / (elevatorMaxHeight - elevatorMinHeight)));

    setHeightRange(intakeLeftStart, intakeLeftEnd, intakeLeftHeightColor, normalizedHeight);
    setHeightRange(intakeRightStart, intakeRightEnd, intakeRightHeightColor, normalizedHeight);
    setHeightRange(elevatorLeftStart, elevatorLeftEnd, elevatorLeftHeightColor, normalizedHeight);
    setHeightRange(elevatorRightStart, elevatorRightEnd, elevatorRightHeightColor, normalizedHeight);

    led.setData(ledBuffer);
  }

  private void setHeightRange(int start, int end, Color color, double normalizedHeight) {
    int range = end - start + 1;
    int ledsToLight = (int) Math.round(normalizedHeight * range);

    boolean inverted = start == intakeRightStart || start == elevatorRightStart;

    for (int i = 0; i < range; i++) {
      int ledIndex = inverted ? end - i : start + i;
      if (i < ledsToLight) {
        ledBuffer.setRGB(ledIndex, color.red, color.blue, color.green);
      } else {
        ledBuffer.setRGB(ledIndex, 0, 0, 0);
      }
    }
  }

}
