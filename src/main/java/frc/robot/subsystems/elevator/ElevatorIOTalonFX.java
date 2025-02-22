package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {
  // Hardware
  private final TalonFX elevatorMotor;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Motion Magic
  private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);

  // Status Signals
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Angle> position;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Current> supplyCurrent;

  public ElevatorIOTalonFX() {
    elevatorMotor = new TalonFX(13);

    // Config Motor
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 =
        new Slot0Configs()
            .withKP(1.0)
            .withKI(0.03)
            .withKD(0.0)
            .withKS(0.1)
            .withKV(0.001)
            .withKG(0.2)
            .withKA(0)
            .withGravityType(GravityTypeValue.Elevator_Static);
    config.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(600)
            .withMotionMagicAcceleration(1800)
            .withMotionMagicExpo_kV(0.12);
    config.Feedback.SensorToMechanismRatio = 15 / (2 * Math.PI);
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.SoftwareLimitSwitch =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Constants.ELEVATOR_UPPER_THRESHOLD.in(Inches))
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(Constants.ELEVATOR_TARGET_GROUND.in(Inches));
    elevatorMotor.getConfigurator().apply(config);

    position = elevatorMotor.getPosition();
    velocity = elevatorMotor.getVelocity();
    appliedVolts = elevatorMotor.getMotorVoltage();
    torqueCurrent = elevatorMotor.getTorqueCurrent();
    supplyCurrent = elevatorMotor.getSupplyCurrent();
    temp = elevatorMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, temp);
    torqueCurrent.setUpdateFrequency(1000);
    ParentDevice.optimizeBusUtilizationForAll(elevatorMotor);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, torqueCurrent, supplyCurrent, temp)
            .isOK();

    inputs.motorConnected = connected;
    inputs.positionInch = position.getValueAsDouble();
    inputs.velocityInchPerSec = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void runOpenLoop(double output) {
    elevatorMotor.set(output);
  }

  @Override
  public void runVolts(double volts) {
    elevatorMotor.set(volts);
  }

  @Override
  public void stop() {
    elevatorMotor.stopMotor();
    ;
  }

  @Override
  public void runPosition(Distance pos) {
    elevatorMotor.setControl(motionRequest.withPosition(pos.in(Inches)));
  }
}
