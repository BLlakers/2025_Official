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

   public static double Down = 0;
   public static double Troph = -2.5;
   public static double L2 = -5.8;
   public static double AlgaeL3 = -10.2;
   public static double L3 = -13.7;
   public static double AlgaeL4 = -17;
   public static double L4 = -24.2;
   
   public static double ElevatorGearRatio = 375;
   private double marginOfError = 1;
   private double elevatorPositionConversionFactor = 1.6*Math.PI; // 1.6 * Math.PI = Distance per rotation
   private double elevatorVelocityConversionFactor = 1; 
   private double desiredPos;
   private elevatorState Estate =elevatorState.Down;
   private double elevDecelerateOffset = 5.6;
   
    //A motor to rotate up and down
   private SparkMax m_ElevatorMotor = new SparkMax(Constants.Port.m_ElevatorMtrC, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

   private DigitalInput m_ElevatorLimitSwitchTop = new DigitalInput(6);
   private DigitalInput m_ElevatorLimitSwitchBottom = new DigitalInput(7);
public Boolean AtBottom = true;
   private SparkMaxConfig m_ElevatorConfig = new SparkMaxConfig();

    public ElevatorMechanism() {
        m_ElevatorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(1.0,0,0);
         m_ElevatorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        m_ElevatorConfig.alternateEncoder //TODO MAKE SURE TO USE RIGHT TYPE OF ENCODER WHEN DOING CONFIGS!
            .positionConversionFactor(elevatorPositionConversionFactor)
            .velocityConversionFactor(elevatorVelocityConversionFactor)
            .countsPerRevolution(8192);
             m_ElevatorMotor.configure(m_ElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

             ResetPosition();
             }

    public void ElevatorMotorUp() {
        m_ElevatorMotor.set(.95);
    }

    public void ResetPosition() {
        m_ElevatorMotor.getAlternateEncoder().setPosition(0);
    }
    public Command ResetPositionCMD(){
        return this.runOnce(this::ResetPosition);
    }

    public void ElevatorMotorDown() {
            m_ElevatorMotor.set(-.95);
        }
    public boolean ElevatorLimitSwitchTop(){
        return m_ElevatorLimitSwitchTop.get();
    }

    public boolean ElevatorLimitSwitchBottom(){
        return m_ElevatorLimitSwitchBottom.get();
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

    public double getElevatorDecelerateRatio(){
        return 1 - ((getElevatorEncoderPos())/(L4 -elevDecelerateOffset));
    }


    public boolean ElevatorAtPos(){
        return getElevatorEncoderPos() < desiredPosGet() + marginOfError && getElevatorEncoderPos() > desiredPosGet() - marginOfError;
    }

    public Command ElevatorUpLimitCmd() {
        return this.runEnd(this::ElevatorMotorUp, this::ElevatorMotorStop).until(() -> ElevatorLimitSwitchTop());
    }

    public Command ElevatorDownLimitCmd() {
        return this.runEnd(this::ElevatorMotorDown, this::ElevatorMotorStop).until(() -> ElevatorLimitSwitchBottom());
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
            desiredPos = L4;
        } else if (Estate == elevatorState.L3) {
            desiredPos = L3;
        } else if (Estate == elevatorState.L2) {
            desiredPos = L2;
        } else if (Estate == elevatorState.Troph) {
            desiredPos =  Troph;
        } else if (Estate == elevatorState.Down) {
            desiredPos = Down;
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
   
    public void ResetElevatorEnc(){
        if (ElevatorLimitSwitchBottom() == true){
            m_ElevatorMotor.getAlternateEncoder().setPosition(0);
            AtBottom = true;
        } else {
            AtBottom = false; 
        }
    }
public elevatorState getEstate(){
    return this.Estate;
}
public void periodic(){
   
}
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Elevator/Position", () -> getElevatorEncoderPos(), null);
    builder.addBooleanProperty("Elevator/LimitSwitchTop", this::ElevatorLimitSwitchTop, null);
    builder.addBooleanProperty("Elevator/LimitSwitchBottom", this::ElevatorLimitSwitchBottom, null);
    builder.addBooleanProperty("Elevator/AtPos", this::ElevatorAtPos, null);
    builder.addDoubleProperty("Elevator/desiredPos", this::desiredPosGet, this::desiredPosSet);
    builder.addStringProperty("Elevator/DesiredLevel", () -> this.Estate.toString(), null);
    builder.addDoubleProperty("Elevator/DecelerateRatio", () -> getElevatorDecelerateRatio(), null);
  }
}
