package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.Constants.Algae;
import frc.robot.subsystems.DriveTrain; 
import frc.robot.subsystems.ElevatorMechanism;

public class ElevatorPID extends Command {
 private DoubleSupplier position;
  private ElevatorMechanism elevator;
  private ProfiledPIDController pid = new ProfiledPIDController(.1, 0, 0, ELEVATOR_CONSTRAINTS);
  private static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(Units.feetToMeters(5),Units.feetToMeters(5));


  public ElevatorPID(ElevatorMechanism e, DoubleSupplier p) {

    elevator = e;
    position = p;
    addRequirements(elevator);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   pid.reset(elevator.getElevatorEncoderPos()); 
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    pid.setGoal(position.getAsDouble());
    double m_elevatorSpeed = pid.calculate(elevator.getElevatorEncoderPos());
    if (pid.atGoal()) {
      m_elevatorSpeed = 0;
    }
    
    if (elevator.ElevatorLimitSwitchTop()) {
      m_elevatorSpeed = 0;
    }
      SmartDashboard.putNumber(elevator.getName() + "ElevatorCommand/Command/elevatorSpeed", m_elevatorSpeed * ElevatorMechanism.ElevatorGearRatio);
      SmartDashboard.putNumber(elevator.getName() + "ElevatorCommand/Command/elevatorPos", elevator.getElevatorEncoderPos());
      elevator.ElevatorMove(m_elevatorSpeed * (ElevatorMechanism.ElevatorGearRatio)); 
    
   
  }
public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    pid.initSendable(builder);
    builder.addDoubleProperty(this.getName() + "/position", position, null);
   
 }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  elevator.ElevatorMotorStop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
