package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.support.Telemetry;
import frc.robot.support.limelight.LimelightUtil;

// some imports no longer needed but leaving them here untill final version

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    String codeVersion = "0.0";
    private PowerDistribution PDH = new PowerDistribution(20, PowerDistribution.ModuleType.kRev);



    @Override
    public void close() {
        super.close();

        // TODO: Evaluate port forwarding teardown
        LimelightUtil.stopPortForwarding();

        Telemetry.shutdown();
    }

    // commit
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        m_robotContainer.m_DriveTrain.ZeroGyro().schedule();
        m_robotContainer.mLedStrand.changeLed(128, 0, 0);
        try (var cam = CameraServer.startAutomaticCapture()) {
            cam.setResolution(100, 100);
            cam.setFPS(60);
        }

        SmartDashboard.putString("Code Version", codeVersion);

        // TODO: Evaluate port forwarding setup
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putData(PDH);
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.m_DriveTrain.m_FieldRelativeEnable = false;
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        m_robotContainer.m_DriveTrain.m_FieldRelativeEnable = true;
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
