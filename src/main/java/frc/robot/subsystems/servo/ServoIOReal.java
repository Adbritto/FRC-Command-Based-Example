package frc.robot.subsystems.servo;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.Constants;

public class ServoIOReal implements ServoIO {
  final Servo servo;

  public ServoIOReal(int port) {
    servo = new Servo(port);
  }

  @Override
  public void set(double output) {
    servo.set(output);
  }

  public void open() {
    set(Constants.FUNNEL_OUTTAKE_POS_ROTS);
  }

  public void close() {
    set(Constants.FUNNEL_CLOSED_POS_ROTS);
  }
}
