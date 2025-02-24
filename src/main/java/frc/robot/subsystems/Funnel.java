package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.breakbeam.BreakbeamIO;
import frc.robot.subsystems.breakbeam.BreakbeamIOInputsAutoLogged;
import frc.robot.subsystems.breakbeam.BreakbeamIOReal;
import frc.robot.subsystems.servo.ServoIO;
import frc.robot.subsystems.servo.ServoIOInputsAutoLogged;
import frc.robot.subsystems.servo.ServoIOReal;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
    private final ServoIOReal servoIO;
    private final BreakbeamIOReal breakbeamIO;
    private final String name;
    private final ServoIOInputsAutoLogged servoIOInputs = new ServoIOInputsAutoLogged();
    private final BreakbeamIOInputsAutoLogged breakbeamIOInputs = new BreakbeamIOInputsAutoLogged();

    private boolean hasCoral = false;

    public Funnel(ServoIOReal servoIO, BreakbeamIOReal breakbeamIO, String name) {
        this.servoIO = servoIO;
        this.breakbeamIO = breakbeamIO;
        this.name = name;


        servoIO.set(Constants.FUNNEL_CLOSED_POS_ROTS);
    }

    @Override
    public void periodic() {
        breakbeamIO.updateInputs(breakbeamIOInputs);

        Logger.processInputs(name + "/breakbeam", breakbeamIOInputs);
        Logger.recordOutput(name  + "/has coral", hasCoral);
    }

    public void setHasCoral() {
        hasCoral = breakbeamIOInputs.get;
    }

    @AutoLogOutput
    public Command intakeCoral() {
        return runOnce(servoIO::close);
    }

    @AutoLogOutput
    public Command outtakeCoral() {
        return startEnd(servoIO::open, servoIO::close);
    }

}
