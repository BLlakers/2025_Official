package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AlgaeMechanism;
import frc.robot.support.Telemetry;
import frc.robot.support.limelight.LimelightHelpers;
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
        // AlgaeMechanism.AUTORunning = true;
        // TODO 1: READ TAG, turn on bot pushed up against the reef wall. make sure to read tag, then move  
        // 
        m_robotContainer = new RobotContainer();
        if (LimelightHelpers.getTV("limelight-frl")){
            if (m_robotContainer.m_LimelightFrl.getCurrentAprilTag().ID ==  17){
                m_robotContainer.m_DriveTrain.navx.setAngleAdjustment(-60);
            }
            if (m_robotContainer.m_LimelightFrl.getCurrentAprilTag().ID ==  18){
                m_robotContainer.m_DriveTrain.navx.setAngleAdjustment(0);
            }
            if (m_robotContainer.m_LimelightFrl.getCurrentAprilTag().ID ==  19){
                m_robotContainer.m_DriveTrain.navx.setAngleAdjustment(60);
            }
            if (m_robotContainer.m_LimelightFrl.getCurrentAprilTag().ID ==  20){
                m_robotContainer.m_DriveTrain.navx.setAngleAdjustment(-120);
            }
            if (m_robotContainer.m_LimelightFrl.getCurrentAprilTag().ID ==  21){
                m_robotContainer.m_DriveTrain.navx.setAngleAdjustment(180);
            }
            if (m_robotContainer.m_LimelightFrl.getCurrentAprilTag().ID ==  22){
                m_robotContainer.m_DriveTrain.navx.setAngleAdjustment(120);
            }
            if (m_robotContainer.m_LimelightFrl.getCurrentAprilTag().ID ==  6){
                m_robotContainer.m_DriveTrain.navx.setAngleAdjustment(-60);
            }
            if (m_robotContainer.m_LimelightFrl.getCurrentAprilTag().ID == 7){
                m_robotContainer.m_DriveTrain.navx.setAngleAdjustment(180);
            }
            if (m_robotContainer.m_LimelightFrl.getCurrentAprilTag().ID ==  8){
                m_robotContainer.m_DriveTrain.navx.setAngleAdjustment(-120);
            }
            if (m_robotContainer.m_LimelightFrl.getCurrentAprilTag().ID ==  9){
                m_robotContainer.m_DriveTrain.navx.setAngleAdjustment(-60);
            }
            if (m_robotContainer.m_LimelightFrl.getCurrentAprilTag().ID ==  10){
                m_robotContainer.m_DriveTrain.navx.setAngleAdjustment(0);
            }
            if (m_robotContainer.m_LimelightFrl.getCurrentAprilTag().ID ==  11){
                m_robotContainer.m_DriveTrain.navx.setAngleAdjustment(120);
            }

            
        }
        m_robotContainer.m_LimelightBack.SetTagIDToTrack(-1);
        m_robotContainer.m_LimelightFrl.SetTagIDToTrack(-1);
        m_robotContainer.m_LimelightFrr.SetTagIDToTrack(-1);
        m_robotContainer.mLedStrand.changeLed(128, 0, 0);
        try {
            var cam = CameraServer.startAutomaticCapture();
            cam.setResolution(100, 100);
            cam.setFPS(60);
        } catch (Exception e) {
            // TODO: handle exception
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
        // AlgaeMechanism.AUTORunning = true;
    }

    @Override
    public void disabledPeriodic() {
        AlgaeMechanism.AUTORunning = true;
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.m_DriveTrain.m_FieldRelativeEnable = false;
        System.out.println(m_robotContainer.m_DriveTrain.m_FieldRelativeEnable);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        AlgaeMechanism.AUTORunning = true;
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {  
        // m_robotContainer.m_DriveTrain.ZeroGyro().schedule();
        AlgaeMechanism.AUTORunning = false;
        m_robotContainer.m_DriveTrain.m_FieldRelativeEnable = true;
        m_robotContainer.m_LimelightBack.SetTagIDToTrack(-1);
        m_robotContainer.m_LimelightFrl.SetTagIDToTrack(-1);
        m_robotContainer.m_LimelightFrr.SetTagIDToTrack(-1);
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
