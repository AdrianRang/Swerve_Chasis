package lib.team3526.control;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

/**
 * A wrapper for SparkPIDController that only updates the controller if the value has changed.
 */
public class LazySparkPID {
    public SparkPIDController controller;

    private double lastReference;
    private ControlType lastControlType;

    private double kP;
    private double kI;
    private double kD;
    private double kIz;
    private double kFF;

    public LazySparkPID(SparkPIDController controller) {
        this.controller = controller;
    }

    public LazySparkPID(LazyCANSparkMax spark) {
        this.controller = spark.getPIDController();
    }

    public void setReference(double reference, ControlType controlType) {
        if (reference != lastReference || controlType != lastControlType) {
            controller.setReference(reference, controlType);
            lastReference = reference;
            lastControlType = controlType;
        }
    }

    public void setP(double kP) {
        if (kP != this.kP) {
            controller.setP(kP);
            this.kP = kP;
        }
    }

    public void setI(double kI) {
        if (kI != this.kI) {
            controller.setI(kI);
            this.kI = kI;
        }
    }

    public void setD(double kD) {
        if (kD != this.kD) {
            controller.setD(kD);
            this.kD = kD;
        }
    }

    public void setIZone(double kIz) {
        if (kIz != this.kIz) {
            controller.setIZone(kIz);
            this.kIz = kIz;
        }
    }

    public void setFF(double kFF) {
        if (kFF != this.kFF) {
            controller.setFF(kFF);
            this.kFF = kFF;
        }
    }
}
