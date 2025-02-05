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
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorMechanism;

public class ElevatorPID extends Command {
 private double position;
private ElevatorMechanism elevator;
private ProfiledPIDController pid = new ProfiledPIDController(1, 0, 0, ELEVATOR_CONSTRAINTS);
private static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);


  public ElevatorPID(ElevatorMechanism e, double p) {
  elevator = e;
  position = p;
    addRequirements(elevator);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    pid.setGoal(position);
  
    double m_elevatorSpeed = pid.calculate(elevator.getElevatorEncoderPos());
    if (pid.atGoal()) {
      m_elevatorSpeed = 0;
    } 
      elevator.ElevatorMove(m_elevatorSpeed);
    
   
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
