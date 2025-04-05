package frc.robot.subsystems.leds;

public interface ILed {
  void setBlinkFrequency(double frequency);

  void turnOff();

  void setSolidColor(Color color);

  void setBlink(Color color, int numberOfBlinks, Color colorAfterBlink);

  void setBlink(Color color, int numberOfBlinks);

  void setBlink(Color color);

  void setRainbow();

}
