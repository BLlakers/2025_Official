package frc.robot.support;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotBase;

import static java.util.Objects.isNull;

public class SparkMaxEncoderStrategy {

    private final SparkMax motor;

    private SparkRelativeEncoderSim lazyEncoderSim;

    public SparkMaxEncoderStrategy(final SparkMax motor) {
        this.motor = motor;
    }

    public void advancePosition() {
        if (RobotBase.isSimulation()) {
            if (isNull(this.lazyEncoderSim)) {
                this.lazyEncoderSim = new SparkRelativeEncoderSim(this.motor);
            }
            this.lazyEncoderSim.setPosition(this.lazyEncoderSim.getPosition() + 2.0);
        } else {
            this.motor.getEncoder().setPosition(this.motor.getEncoder().getPosition() + 2.0);
        }
    }

    public double getPosition() {
        if (RobotBase.isSimulation()) {
            if (isNull(this.lazyEncoderSim)) {
                this.lazyEncoderSim = new SparkRelativeEncoderSim(this.motor);
            }
            return this.lazyEncoderSim.getPosition();
        } else {
            return this.motor.getEncoder().getPosition();
        }
    }

    public double getVelocity() {
        if (RobotBase.isSimulation()) {
            if (isNull(lazyEncoderSim)) {
                this.lazyEncoderSim = new SparkRelativeEncoderSim(this.motor);
            }
            return this.lazyEncoderSim.getVelocity();
        } else {
            return this.motor.getEncoder().getVelocity();
        }
    }
}
