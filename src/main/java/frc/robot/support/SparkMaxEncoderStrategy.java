package frc.robot.support;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotBase;

import static java.util.Objects.isNull;

public class SparkMaxEncoderStrategy {

    private final SparkMax motor;

    private SparkRelativeEncoderSim encoderSim;

    public SparkMaxEncoderStrategy(final SparkMax motor) {
        this.motor = motor;
    }

    public double getPosition(){
        if(RobotBase.isSimulation()){
            if(isNull(encoderSim)){
                this.encoderSim = new SparkRelativeEncoderSim(this.motor);
            }
            return encoderSim.getPosition();
        }else{
            return motor.getEncoder().getPosition();
        }
    }

    public double getVelocity(){
        // TODO: @avibot1 Implement getVelocity() in a similar way to the above getPosition()
        // NOTE: returning temp value pending correct implementation
        return 0.0;
    }
}
