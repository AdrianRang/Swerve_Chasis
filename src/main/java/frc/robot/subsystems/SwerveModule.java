package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.team3526.constants.PIDFConstants;
import lib.team3526.constants.SwerveModuleOptions;
import lib.team3526.control.LazyCANSparkMax;
import lib.team3526.control.LazySparkPID;
import static edu.wpi.first.units.Units.*;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {
    //! Options for the module
    public final SwerveModuleOptions options;

    //! Motors
    private final LazyCANSparkMax driveMotor;
    private final LazyCANSparkMax turningMotor;

    //! Encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    //! PID Controller for turning
    private final LazySparkPID turningPID;

    //! Absolute encoder
    private final CANcoder absoluteEncoder;

    //! Target state
    private SwerveModuleState targetState = new SwerveModuleState();

    /**
     * Create a new swerve module with the provided options
     * @param options
     */
    public SwerveModule(SwerveModuleOptions options) {
        // Store the options
        this.options = options;

        // Create the motors
        this.driveMotor = new LazyCANSparkMax(options.driveMotorID, MotorType.kBrushless);
        this.turningMotor = new LazyCANSparkMax(options.turningMotorID, MotorType.kBrushless);

        this.turningMotor.setInverted(options.turningMotorInverted);

        // Get and configure the encoders
        this.driveEncoder = this.driveMotor.getEncoder();
        this.driveEncoder.setPositionConversionFactor(Constants.SwerveDrive.PhysicalModel.kDriveEncoder_RotationToMeter); 
        this.driveEncoder.setVelocityConversionFactor(Constants.SwerveDrive.PhysicalModel.kDriveEncoder_RPMToMeterPerSecond);
        this.driveMotor.setInverted(options.driveMotorInverted);

        this.turningEncoder = this.turningMotor.getEncoder();
        this.turningEncoder.setPositionConversionFactor(Constants.SwerveDrive.PhysicalModel.kTurningEncoder_RotationToRadian); 
        this.turningEncoder.setVelocityConversionFactor(Constants.SwerveDrive.PhysicalModel.kTurningEncoder_RPMToRadianPerSecond);

        this.turningPID = new LazySparkPID(this.turningMotor.getPIDController());
        PIDFConstants.applyToSparkPIDController(this.turningPID.controller, Constants.SwerveDrive.SwerveModules.kTurningPIDConstants);
        this.turningPID.controller.setPositionPIDWrappingMinInput(0);
        this.turningPID.controller.setPositionPIDWrappingMaxInput(2 * Math.PI);
        this.turningPID.controller.setPositionPIDWrappingEnabled(true);

        // Configure the absolute encoder
        this.absoluteEncoder = new CANcoder(options.absoluteEncoderDevice.getDeviceID(), options.absoluteEncoderDevice.getCanbus());

        // Reset the encoders
        resetEncoders();
        
    }

    /**
     * Get the absolute encoder turn position
     * @return
     */
    public Measure<Angle> getAbsoluteEncoderPosition() {
        return Radians.of((absoluteEncoder.getAbsolutePosition().refresh().getValue() * 2 * Math.PI) * (this.options.absoluteEncoderInverted ? -1.0 : 1.0));
    }

    /**
     * Reset the drive encoder (set the position to 0)
     */
    public void resetDriveEncoder() {
        this.driveEncoder.setPosition(0);
    }

    /**
     * Reset the turning encoder (set the position to the absolute encoder's position)
     */
    public void resetTurningEncoder() {
        this.turningEncoder.setPosition(getAbsoluteEncoderPosition().in(Radians));
    }
    
    /**
     * Reset the drive and turning encoders
     */
    public void resetEncoders() {
        resetDriveEncoder();
        resetTurningEncoder();
    }

    /**
     * Get the current angle of the module
     * @return
     */
    public Measure<Angle> getAngle() {
        return Radians.of(this.turningEncoder.getPosition() % (2 * Math.PI));
    }

    /**
     * Set the target state of the module
     * @param state The target state
     */
    public void setTargetState(SwerveModuleState state) {
        setTargetState(state, false);
    }

    /**
     * Set the target state of the module
     * @param state The target state
     * @param force If true, the module will ignore the current speed and turn to the target angle
     */
    public void setTargetState(SwerveModuleState state, boolean force) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001 || force) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, Rotation2d.fromRadians(getAngle().in(Radians)));

        this.targetState = state;

        driveMotor.set(state.speedMetersPerSecond / Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond));
        turningPID.setReference(state.angle.getRadians(), ControlType.kPosition);
    }

    /**
     * Stop the module (set the speed of the motors to 0)
     */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    /**
     * Get the target state of the module
     * @return
     */
    public SwerveModuleState getTargetState() {
        return this.targetState;
    }

    /**
     * Get the real state of the module
     * @return
     */
    public SwerveModuleState getRealState() {
        return new SwerveModuleState(
            this.driveEncoder.getVelocity(),
            Rotation2d.fromRadians(this.getAngle().in(Radians))
        );
    }

    /**
     * Get the position of the module
     * @return
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveEncoder.getPosition(),
            Rotation2d.fromRadians(this.getAngle().in(Radians))
        );
    }

    public void periodic() {
        Logger.recordOutput("SwerveDrive/" + this.options.name + "/MotEncoderDeg", this.getAngle().in(Degrees));
        Logger.recordOutput("SwerveDrive/" + this.options.name + "/AbsEncoderDeg", this.getAbsoluteEncoderPosition().in(Degrees));
        Logger.recordOutput("SwerveDrive/" + this.options.name + "/RealState", this.getRealState());
        Logger.recordOutput("SwerveDrive/" + this.options.name + "/TargetState", this.getTargetState());
    }
}
