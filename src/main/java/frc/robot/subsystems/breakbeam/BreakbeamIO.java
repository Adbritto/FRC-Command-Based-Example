package frc.robot.subsystems.breakbeam;

import org.littletonrobotics.junction.AutoLog;

public interface BreakbeamIO {
  @AutoLog
  public static class BreakbeamIOInputs {
    public boolean get = false;
  }

  public void updateInputs(BreakbeamIOInputsAutoLogged inputs);
}
