package frc.robot.subsystems.servo;

import edu.wpi.first.wpilibj.Servo;

public class ServoIOReal implements ServoIO {
  final Servo servo;

  public ServoIOReal(int port) {
    servo = new Servo(port);
  }

  @Override
  public void set(double output) {
    servo.set(output);
  }
}
