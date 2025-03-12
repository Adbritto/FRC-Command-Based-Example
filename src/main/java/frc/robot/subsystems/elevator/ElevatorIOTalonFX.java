package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {
  // Hardware
  private final TalonFX elevatorMotor = new TalonFX(13);

  // Motion Magic
  private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);

  // Status Signals
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Angle> position;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Current> supplyCurrent;

  private final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              Units.inchesToMeters(50.21), Units.inchesToMeters(747.43)));
  private TrapezoidProfile.State lastState = new TrapezoidProfile.State();

  private final LinearSystem<N2, N1, N2> elevatorSstem =
      LinearSystemId.createElevatorSystem(
          DCMotor.getKrakenX60(1), Units.lbsToKilograms(5), Units.inchesToMeters(1.45 / 2), 9.0);

  @SuppressWarnings("unchecked")
  private final KalmanFilter<N2, N1, N1> kalmanFilter =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N1(),
          (LinearSystem<N2, N1, N1>) elevatorSstem.slice(0),
          VecBuilder.fill(Units.inchesToMeters(2), Units.inchesToMeters(40)),
          VecBuilder.fill(0.001),
          0.020);

  @SuppressWarnings("unchecked")
  private final LinearQuadraticRegulator<N2, N1, N1> lqrCon =
      new LinearQuadraticRegulator<>(
          (LinearSystem<N2, N1, N1>) elevatorSstem.slice(0),
          VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(10.0)),
          VecBuilder.fill(12.0),
          0.020);

  @SuppressWarnings("unchecked")
  private final LinearSystemLoop<N2, N1, N1> loop =
      new LinearSystemLoop<>(
          (LinearSystem<N2, N1, N1>) elevatorSstem.slice(0), lqrCon, kalmanFilter, 12.0, 0.020);

  public ElevatorIOTalonFX() {
    // Config Motor
    // Config
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = 9 / (1.45 * Math.PI);
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
    elevatorMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    elevatorMotor.stopMotor();
  }

  @Override
  public void runPosition(Distance pos) {
    TrapezoidProfile.State goal = new TrapezoidProfile.State(pos.in(Inches), 0.0);
    lastState = profile.calculate(0.020, lastState, goal);
    loop.setNextR(lastState.position, lastState.velocity);
    loop.correct(VecBuilder.fill(elevatorMotor.getPosition().getValueAsDouble()));
    loop.predict(0.020);
    double voltage = loop.getU(0);
    elevatorMotor.setVoltage(voltage);
  }

  @Override
  public void reset() {
    loop.reset(
        VecBuilder.fill(
            elevatorMotor.getPosition().getValueAsDouble(),
            elevatorMotor.getVelocity().getValueAsDouble()));
    lastState =
        new TrapezoidProfile.State(
            elevatorMotor.getPosition().getValueAsDouble(),
            elevatorMotor.getVelocity().getValueAsDouble());
  }
}
