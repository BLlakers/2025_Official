package frc.robot.subsystems;

import java.util.function.DoubleConsumer;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

enum elevatorState {
    Down,
    Troph,
    L2,
    L3,
    L4
}

public class ElevatorMechanism extends SubsystemBase{
   public static double ElevatorGearRatio = 3;
   private double marginOfError = 1;
   private double elevatorPositionConversionFactor = 1.6*Math.PI ; // 1.6 * Math.PI = Distance per rotation
   private double elevatorVelocityConversionFactor = 1; 
   private double desiredPos;
   private elevatorState Estate =elevatorState.Down;
   
    //A motor to rotate up and down
   private SparkMax m_ElevatorMotor = new SparkMax(Constants.Port.m_ElevatorMtrC, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

   private DigitalInput m_ElevatorLimitSwitch = new DigitalInput(6);
    
   private SparkMaxConfig m_ElevatorConfig = new SparkMaxConfig();

    public ElevatorMechanism() {
        m_ElevatorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(1.0,0,0);
         m_ElevatorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        m_ElevatorConfig.alternateEncoder //TODO MAKE SURE TO USE RIGHT TYPE OF ENCODER WHEN DOING CONFIGS!
            .positionConversionFactor(elevatorPositionConversionFactor)
            .velocityConversionFactor(elevatorVelocityConversionFactor)
            .countsPerRevolution(8192);
             m_ElevatorMotor.configure(m_ElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
        return getElevatorEncoderPos() < desiredPosGet() + marginOfError && getElevatorEncoderPos() > desiredPosGet() - marginOfError;
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
    public void desiredPosSet(double s){
        desiredPos = s;
    }

    public void MoveDesiredPosUp(){
        if (Estate == elevatorState.Down) {
            Estate = elevatorState.Troph;
        } else if (Estate == elevatorState.Troph){
        Estate = elevatorState.L2;
        } else if (Estate == elevatorState.L2) {
        Estate = elevatorState.L3;
        } else if (Estate == elevatorState.L3) {
        Estate = elevatorState.L4;
        } else if (Estate == elevatorState.L4) {
        Estate = elevatorState.L4;
        }

    }
    
    public void MoveDesiredPosDown(){
        if (Estate == elevatorState.L4){
            Estate = elevatorState.L3;
        } else if (Estate == elevatorState.L3) {
            Estate = elevatorState.L2;
        } else if (Estate == elevatorState.L2) {
            Estate = elevatorState.Troph;
        } else if (Estate == elevatorState.Troph) {
            Estate = elevatorState.Down;
        } else if (Estate == elevatorState.Down) {
            Estate = elevatorState.Down;
        } 
    }

    public void ChangeDesiredPos(){
        if (Estate == elevatorState.L4){
            desiredPos = 400;
        } else if (Estate == elevatorState.L3) {
            desiredPos = 300;
        } else if (Estate == elevatorState.L2) {
            desiredPos = 200;
        } else if (Estate == elevatorState.Troph) {
            desiredPos =  100;
        } else if (Estate == elevatorState.Down) {
            desiredPos = 0;
        } 
    }

   


    public Command MovePosUp(){
       
        return runOnce(()-> MoveDesiredPosUp()).andThen(() -> ChangeDesiredPos());
    }

    public Command MovePosDown(){
     
        return runOnce(()-> MoveDesiredPosDown()).andThen(() -> ChangeDesiredPos());
    }
    public double desiredPosGet(){
        return desiredPos;
    }
   


public void periodic(){
   System.out.println(Estate);
}
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    ElevatorUpCmd().setName("ElevatorUpCmd");
    builder.addDoubleProperty("Elevator/Position", () -> getElevatorEncoderPos(), null);
    builder.addBooleanProperty("Elevator/LimitSwitch", this::ElevatorLimitSwitch, null);
    builder.addBooleanProperty("Elevator/AtPos", this::ElevatorAtPos, null);
    builder.addDoubleProperty("Elevator/desiredPos", this::desiredPosGet, this::desiredPosSet);
    builder.addStringProperty("Elevator/DesiredLevel", () -> this.Estate.toString(), null);
  }
}
