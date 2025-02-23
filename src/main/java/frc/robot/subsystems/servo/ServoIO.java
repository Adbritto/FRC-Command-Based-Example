package frc.robot.subsystems.servo;

import org.littletonrobotics.junction.AutoLog;

public interface ServoIO {
  @AutoLog
  public static class ServoIOInputs {}

  public void set(double output);
}
