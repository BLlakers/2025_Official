package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class SwerveDriveCommand extends Command {
 private DoubleSupplier m_leftY;
 private DoubleSupplier m_leftX;
 private DoubleSupplier m_rightX;
 private DoubleSupplier m_AccelerateRT;
 private DriveTrain m_DriveTrain;

  private BooleanSupplier m_RunHalfSpeed;
  private DoubleSupplier m_ElevatorDecelerate;

  private static final double kDriveMaxSpeed = 0.85 * DriveTrain.kMaxSpeed;
  private static final double kTurnMaxSpeed = 0.5 * DriveTrain.kMaxTurnAngularSpeed;

  public SwerveDriveCommand(
      DoubleSupplier _leftY,
      DoubleSupplier _leftX,
      DoubleSupplier _rightX,
      DoubleSupplier _AccelerateRT,
      DriveTrain _dTrain) {
    m_leftY = _leftY;
    m_leftX = _leftX;
    m_rightX = _rightX;
    m_DriveTrain = _dTrain;
    m_AccelerateRT = _AccelerateRT;
    m_RunHalfSpeed = () -> false;
    m_ElevatorDecelerate = () -> 1.0;
    addRequirements(m_DriveTrain);
  }

  public SwerveDriveCommand(
      DoubleSupplier _leftY,
      DoubleSupplier _leftX,
      DoubleSupplier _rightX,
      DoubleSupplier _AccelerateRT,
      DriveTrain _dTrain,
      BooleanSupplier _halfSpeedCondition) {
    m_leftY = _leftY;
    m_leftX = _leftX;
    m_rightX = _rightX;
    m_DriveTrain = _dTrain;
    m_AccelerateRT = _AccelerateRT;
    m_RunHalfSpeed = _halfSpeedCondition;
    m_ElevatorDecelerate = () -> 1.0;
    addRequirements(m_DriveTrain);
  }

  public SwerveDriveCommand(
    DoubleSupplier _leftY,
    DoubleSupplier _leftX,
    DoubleSupplier _rightX,
    DoubleSupplier _AccelerateRT,
    DoubleSupplier _ElevatorDecelerate,
    DriveTrain _dTrain,
    BooleanSupplier _halfSpeedCondition) {
  m_leftY = _leftY;
  m_leftX = _leftX;
  m_rightX = _rightX;
  m_DriveTrain = _dTrain;
  m_AccelerateRT = _AccelerateRT;
  m_RunHalfSpeed = _halfSpeedCondition;
  m_ElevatorDecelerate = _ElevatorDecelerate;
  addRequirements(m_DriveTrain);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double RT, x, y, rot, Elev;
    double AccelerateRT = m_AccelerateRT.getAsDouble();
    double leftX = m_leftX.getAsDouble();
    double leftY = m_leftY.getAsDouble();
    double rightX = m_rightX.getAsDouble();
    double DecelerateElev = m_ElevatorDecelerate.getAsDouble();

    // Finds the X Value of the Left Stick on the Controller and Takes Care of
    // Joystick Drift
    x = MathUtil.applyDeadband(-leftX, Constants.Controller.deadzone);

    // Finds the Y Value of the Left Stick on the Controller and Takes Care of
    // Joystick Drift
    y = MathUtil.applyDeadband(-leftY, Constants.Controller.deadzone);

    // Finds the X Value of the Right Stick on the Controller and Takes Care of
    // Joystick Drift
    rot = MathUtil.applyDeadband(-rightX, Constants.Controller.deadzone);

    if(Constants.CurrentDriver.currentDriver == "Asa"){
      RT = 1.0;
    } else {
      RT = AccelerateRT;
    }

    Elev = DecelerateElev;

    double normalizingFactor = Math.hypot(x, y);
    if (normalizingFactor > 0) {
      x /= normalizingFactor;
      y /= normalizingFactor;
    }
    double xSpeed = y * SwerveDriveCommand.kDriveMaxSpeed * RT * Elev;
    double ySpeed = x * SwerveDriveCommand.kDriveMaxSpeed * RT* Elev;
    double rotSpeed = rot * SwerveDriveCommand.kTurnMaxSpeed * Elev;

    if (m_RunHalfSpeed.getAsBoolean() == true) {
      xSpeed /= 2;
      ySpeed /= 2;
      rotSpeed /= 2;
    }
    
    SmartDashboard.putNumber("DriveTrain/Controller/Command/X Speed", xSpeed);
    SmartDashboard.putNumber("DriveTrain/Controller/Command/Y Speed", ySpeed);

    m_DriveTrain.drive(xSpeed, ySpeed, rotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
