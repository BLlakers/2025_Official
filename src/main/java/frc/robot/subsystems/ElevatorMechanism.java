package frc.robot.subsystems;

import java.util.function.DoubleConsumer;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
   public double m_elevatorSpeed;
   public static double Down = 0;
   public static double Troph = -2.5;
   public static double L2 = -5.8;
   public static double AlgaeL3 = -10.2;
   public static double L3 = -13.7;
   public static double AlgaeL4 = -17;
   public static double L4 = -24.5;
   public static boolean IsMoving;
   public static double ElevatorGearRatio = 375;
   private double marginOfError = 1;
   private double elevatorPositionConversionFactor = 1.6*Math.PI; // 1.6 * Math.PI = Distance per rotation
   private double elevatorVelocityConversionFactor = 1; 
   private double desiredPos;
   private elevatorState Estate =elevatorState.Down;
   private double elevDecelerateOffset = 5.6;
   //public double position;
   public double elevatorPosition;
   private ProfiledPIDController pid = new ProfiledPIDController(.0007, 0, 0, ELEVATOR_CONSTRAINTS);
  private static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(Units.feetToMeters(140),Units.feetToMeters(125));


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
        if(!m_ElevatorLimitSwitchTop.get()){
            m_ElevatorMotor.set(.95);
        }else {
            m_ElevatorMotor.set(0);
        }

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

public void initElevatorPID(){
    pid.reset(getElevatorEncoderPos());
}

public void setElevatorPIDPos(double desiredPos){
    elevatorPosition = desiredPos;
}

public void pid(double position){
    pid.setGoal(position);
    m_elevatorSpeed = pid.calculate(getElevatorEncoderPos());
    if (pid.atGoal()) {
      m_elevatorSpeed = 0;
    }
    
    if (ElevatorLimitSwitchTop() && m_elevatorSpeed < 0) {
      m_elevatorSpeed = 0;
    }

    if (ElevatorLimitSwitchBottom() && m_elevatorSpeed > 0) {
        m_elevatorSpeed = 0;
      }
    
    ElevatorMove(m_elevatorSpeed*ElevatorMechanism.ElevatorGearRatio); 
}

public boolean atPIDGoal(){
    return pid.atGoal();
}

@Override
public void periodic(){
    pid(elevatorPosition);   
}
   


  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(getName() + "ElevatorCommand/Command/elevatorSpeed", ()-> m_elevatorSpeed * ElevatorMechanism.ElevatorGearRatio, null);
    builder.addDoubleProperty(getName() + "ElevatorCommand/Command/elevatorDesirePIDPos", () -> elevatorPosition, null);
    builder.addDoubleProperty("Elevator/Position", () -> getElevatorEncoderPos(), null);
    builder.addBooleanProperty("Elevator/LimitSwitchTop", this::ElevatorLimitSwitchTop, null);
    builder.addBooleanProperty("Elevator/LimitSwitchBottom", this::ElevatorLimitSwitchBottom, null);
    builder.addBooleanProperty("Elevator/AtPos", this::ElevatorAtPos, null);
    builder.addDoubleProperty("Elevator/desiredPos", this::desiredPosGet, this::desiredPosSet);
    builder.addStringProperty("Elevator/DesiredLevel", () -> this.Estate.toString(), null);
    builder.addDoubleProperty("Elevator/DecelerateRatio", () -> getElevatorDecelerateRatio(), null);
  }
}
