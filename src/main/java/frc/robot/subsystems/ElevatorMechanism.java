package frc.robot.subsystems;

import java.util.function.DoubleConsumer;

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

   private double elevatorPositionConversionFactor = 1; //1.6 * Math.PI; 1.6 * Math.PI Distance per rotation
   private double elevatorVelocityConversionFactor = 1; 
   private double desiredPos;
    //A motor to rotate up and down
   private SparkMax m_ElevatorMotor = new SparkMax(Constants.Port.m_ElevatorMtrC, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

   private DigitalInput m_ElevatorLimitSwitch = new DigitalInput(6);
    
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
            m_ElevatorMotor.set(-.85);
        }
    public boolean ElevatorLimitSwitch(){
        return m_ElevatorLimitSwitch.get();
    }

    public void ElevatorMotorStop() {
        m_ElevatorMotor.set(0);
    }
    public void ElevatorMove(double d){
        m_ElevatorMotor.set(-d);
    }
    public double getElevatorEncoderPos(){
        return m_ElevatorMotor.getAlternateEncoder().getPosition();
    }


    public boolean ElevatorAtPos(){
        return getElevatorEncoderPos() > 100;
    }

    public Command ElevatorUpCmd() {
        return this.runEnd(this::ElevatorMotorUp, this::ElevatorMotorStop);
    }

    public Command ElevatorDownCmd() {
        return this.runEnd(this::ElevatorMotorDown, this::ElevatorMotorStop);
    }

    public Command ElevatorStopCmd() {
        return this.runOnce(this::ElevatorMotorStop);
    }
    public double desiredPosGet(){
        System.out.println(desiredPos);
        return desiredPos;
    }
    public void desiredPosSet(double s){
        desiredPos = s;
    }

    public Command SetPosUp(){
        return runOnce(()-> desiredPosSet(100));
    }

    public Command SetPosDown(){
        return runOnce(()-> desiredPosSet(0));
    }


public void periodic(){
}
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    ElevatorUpCmd().setName("ElevatorUpCmd");
    builder.addDoubleProperty("Elevator/Position", () -> getElevatorEncoderPos(), null);
    builder.addBooleanProperty("Elevator/LimitSwitch", this::ElevatorLimitSwitch, null);
    builder.addBooleanProperty("Elevator/AtPos", this::ElevatorAtPos, null);
    builder.addDoubleProperty("Elevator/desiredPos", this::desiredPosGet, this::desiredPosSet);
  }
}
