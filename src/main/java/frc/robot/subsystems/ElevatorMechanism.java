package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorMechanism extends SubsystemBase{
   private double elevatorPositionConversionFactor = 1/625;
   private double elevatorVelocityConversionFactor = 1;

    //A motor to rotate up and down
   private SparkMax m_ElevatorMotor = new SparkMax(Constants.Port.m_ElevatorMtrC, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

   private DigitalInput m_ElevatorLimitSwitch = new DigitalInput(0);
    
   private SparkMaxConfig m_ElevatorConfig = new SparkMaxConfig();

    public ElevatorMechanism() {
         m_ElevatorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        m_ElevatorConfig.encoder
            .positionConversionFactor(elevatorPositionConversionFactor)
            .velocityConversionFactor(elevatorVelocityConversionFactor);
        m_ElevatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pid(1.0,0,0);
    }

    public void ElevatorMotorUp() {
        m_ElevatorMotor.set(.85);
    }

    public void ElevatorMotorDown() {
        if (m_ElevatorLimitSwitch.get()) {
            m_ElevatorMotor.set(0);
        } else {
            m_ElevatorMotor.set(-.85);
        }
    }

    public void ElevatorMotorStop() {
        m_ElevatorMotor.set(0);
    }

    public double getElevatorEncoderPos(){
        return m_ElevatorMotor.getAlternateEncoder().getPosition();
    }

    public boolean ElevatorAtPos(){
        return getElevatorEncoderPos() > 100;
    }

    public Command ElevatorUpCmd() {
        return this.runOnce(this::ElevatorMotorUp);
    }

    public Command ElevatorDownCmd() {
        return this.runOnce(this::ElevatorMotorDown);
    }

    public Command ElevatorStopCmd() {
        return this.runOnce(this::ElevatorMotorStop);
    }


    
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Elevator/Position", () -> getElevatorEncoderPos(), null);
  }
}
