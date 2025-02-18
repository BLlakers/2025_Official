package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeMechanism extends SubsystemBase{
    
  private ProfiledPIDController pid = new ProfiledPIDController(.1 , 0, 0, ALGAE_CONSTRAINTS);
  private static final TrapezoidProfile.Constraints ALGAE_CONSTRAINTS = new TrapezoidProfile.Constraints(Units.feetToMeters(10),Units.feetToMeters(2.5));

  double m_AlgaeSpeed;
    public static double posDown = 0;
    public static double posUp = 30; 

    public static double GEAR_RATIO = 1;
   double algaePositionConversionFactor =
   2 * Math.PI / AlgaeMechanism.GEAR_RATIO; // revolutions -> radians
private double algaeVelocityConversionFactor = 1;
private DigitalInput m_AlgaeLimitSwitchBottom = new DigitalInput(8);
 //private DigitalInput m_AlgaeLimitSwitchTop = new DigitalInput(7);
    //A motor to rotate up and down
   private SparkMax m_AlgaeMotor = new SparkMax(Constants.Algae.m_AlgaeMtrC, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
   private SparkMaxConfig m_AlgaeConfig = new SparkMaxConfig();
private AlgaeIntake algaeIntake = new AlgaeIntake();
    public AlgaeMechanism(){
        m_AlgaeConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(1.0,0,0);
        m_AlgaeConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        m_AlgaeConfig.alternateEncoder
            .positionConversionFactor(algaePositionConversionFactor)
            .velocityConversionFactor(algaeVelocityConversionFactor)
            .countsPerRevolution(8192);
       
        pid.setTolerance(.1);
       

            m_AlgaeMotor.configure(m_AlgaeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        }

        public AlgaeIntake AlgaeIntakeGet(){
            return algaeIntake;
        }
    public void AlgaeForward() {
        m_AlgaeMotor.set(.85);
    }

    public void AlgaeBackward() {
        m_AlgaeMotor.set(-.85);
    }

    public void AlgaeStop() {
        m_AlgaeMotor.set(0);
    }

   
    
    public double getAlgaeEncoderPos(){
        return m_AlgaeMotor.getAlternateEncoder().getPosition();
    }

    public Command AlgaeForwardCmd() {
        return this.runOnce(this::AlgaeForward);
    }

    public Command AlgaeBackwardCmd() {
        return this.runOnce(this::AlgaeBackward);
    }

    public Command AlgaeStopCmd() {
        return this.runOnce(this::AlgaeStop);
    }

   
    public void ResetAlgaeEnc() {
       if (m_AlgaeLimitSwitchBottom.get() == true){ 
        m_AlgaeMotor.getAlternateEncoder().setPosition(0);
        }
    }

    
  public Rotation2d getAlgaePos() {
        return Rotation2d.fromRadians(m_AlgaeMotor.getAlternateEncoder().getPosition());
    }
   


public void AlgaePID(double desPosition){
    pid.setGoal(desPosition);
    m_AlgaeSpeed = pid.calculate(getAlgaePos().getRadians());
    if (pid.atGoal()) {
      m_AlgaeSpeed = 0;
    } 
    AlgaeMove(m_AlgaeSpeed);
    
}
public void resetAlgaePid(){
    pid.reset(getAlgaeEncoderPos());
}
    


   
    public void AlgaeMove(double m) {
        m_AlgaeMotor.set(m);
    }


    public Command AlgaePIDUp(){
        return resetAlgaePIDCmd().andThen(runEnd(() -> AlgaePID(0), this::AlgaeStop).onlyWhile(() -> !CheckAlgaePID()));
    }

    public Command AlgaePIDDown(){
        return resetAlgaePIDCmd().andThen(runEnd(() -> AlgaePID(30), this::AlgaeStop).onlyWhile(() ->!CheckAlgaePID()));
    }
    public boolean CheckAlgaePID(){
            return pid.atGoal();
        }
    
    public Command resetAlgaePIDCmd(){
      return this.runOnce(() -> resetAlgaePid());
    }

    public Command runAlgaeDown(){
        return Commands.parallel(runEnd(null, null), algaeIntake.RunIntake());
    }
 @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(this.getName() + "/Algae/Position/Rad", () -> getAlgaePos().getRadians(), null);
  builder.addDoubleProperty(this.getName() + "/Algae/Position/EncoderPos", () -> getAlgaeEncoderPos(), null);
  builder.addDoubleProperty(this.getName() + "/Algae/Position/Goal", () -> pid.getGoal().position, null);
  builder.addBooleanProperty(this.getName() + "/Algae/Position/Goal", () -> pid.atSetpoint(), null);
  builder.addDoubleProperty(this.getName() + "/Algae/Position/Speed", () -> m_AlgaeSpeed, null);
  }
}
