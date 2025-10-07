package frc.robot.subsystems.Drivetrain;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.ModuleOptimizer;
import frc.robot.Constants;

public class newSwerveModule extends SubsystemBase {
    
    public int moduleID;

    private Rotation2d lastAngle;
    private Rotation2d angleOffset;
    private Rotation2d encoderOffset;

    private SparkMax driveMotor;
    private SparkMax steerMotor;

    private RelativeEncoder steerMotorEncoder;
    private RelativeEncoder driveMotorEncoder;
    private CANcoder angleEncoder;

    private final SparkClosedLoopController steerController;
    private final SparkClosedLoopController driveController;

    private final SimpleMotorFeedforward feedforward =  new SimpleMotorFeedforward(Constants.Swerve.driveKS, 
                                                                                   Constants.Swerve.driveKV, 
                                                                                   Constants.Swerve.driveKA);    
    
    public newSwerveModule(int moduleID, SwerveModuleConstants moduleConstants){
        
        this.moduleID = moduleID;
        angleOffset = moduleConstants.angleOffset;
        encoderOffset = moduleConstants.encoderOffset;

        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();
        angleEncoder.getAbsolutePosition().waitForUpdate(0.2);

        steerMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        steerMotorEncoder = steerMotor.getEncoder();
        steerController = steerMotor.getClosedLoopController();
        configSteerMotor();

        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveMotorEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        desiredState = ModuleOptimizer.optimize(desiredState, getState().angle);
    
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void resetToAbsolute() {

        double absolutePosition = getCANCoderAngle().getDegrees() - angleOffset.getDegrees();

        steerMotorEncoder.setPosition(absolutePosition);

    }

    private void configSteerMotor() {

        SparkMaxConfig steerMotorConfig = new SparkMaxConfig();
        ClosedLoopConfig steerMotorCLConfig = new ClosedLoopConfig();
        EncoderConfig steerMotorEncoderConfig = new EncoderConfig();
        

        steerMotorEncoderConfig.positionConversionFactor(Constants.Swerve.angleConversionFactor);
        
        steerMotorCLConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                          .p(0.02).i(0.0).d(0.0);

        steerMotorConfig.idleMode(IdleMode.kBrake)
                        .inverted(Constants.Swerve.angleInvert)
                        .voltageCompensation(Constants.Swerve.voltageComp)
                        .smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit)
                        .apply(steerMotorCLConfig)
                        .apply(steerMotorEncoderConfig);
        
        steerMotor.configure(steerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        resetToAbsolute();

    }

    private void configDriveMotor() {

        SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
        ClosedLoopConfig driveMotorCLConfig = new ClosedLoopConfig();
        EncoderConfig driveMotorEncoderConfig = new EncoderConfig();

        driveMotorEncoderConfig.positionConversionFactor(Constants.Swerve.driveConversionPositionFactor);

        driveMotorCLConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                          .p(0.2).i(0.0).d(0.0);
        
        driveMotorConfig.idleMode(IdleMode.kBrake)
                        .inverted(Constants.Swerve.angleInvert)
                        .voltageCompensation(Constants.Swerve.voltageComp)
                        .smartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit)
                        .apply(driveMotorCLConfig)
                        .apply(driveMotorEncoderConfig);

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveMotorEncoder.setPosition(0.0);

        
    }

    
    private void configAngleEncoder() {

        var angleEncoderConfigs = new CANcoderConfiguration();
        angleEncoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        angleEncoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

     
        angleEncoder.getConfigurator().apply(angleEncoderConfigs);

    }
    

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

        if (isOpenLoop) {

            double power = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(power);
        
        } 
        else {

            driveController.setReference(feedforward.calculate(desiredState.speedMetersPerSecond), 
                                         ControlType.kPosition, 
                                         ClosedLoopSlot.kSlot0);
              
        }
    }

    private void setAngle(SwerveModuleState desiredState) {

        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.1))
                ? lastAngle
                : desiredState.angle;
    
        steerController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;

    }

    public double getDrivePosition(){

        return driveMotor.getEncoder().getPosition() * Constants.Swerve.driveConversionPositionFactor;

    }


    public Rotation2d getSteerAngle() {

        double angle = steerMotorEncoder.getPosition();
        return Rotation2d.fromDegrees(angle);

    }

    public Rotation2d getCANCoderAngle() {


        double angle = ((angleEncoder.getAbsolutePosition().getValueAsDouble()) * 360.0) - encoderOffset.getDegrees();
        return Rotation2d.fromDegrees(angle);

    }

    public SwerveModulePosition getPosition() {

        return new SwerveModulePosition(driveMotor.getEncoder().getPosition() * Constants.Swerve.driveConversionPositionFactor, getSteerAngle());

    }

    public SwerveModuleState getState() {

        return new SwerveModuleState(driveMotor.getEncoder().getVelocity() * Constants.Swerve.driveConversionVelocityFactor, getSteerAngle());

    }

    public SwerveModuleState getStateEncoder() {

        return new SwerveModuleState(driveMotor.getEncoder().getVelocity(), getCANCoderAngle());

    }

    @Override
    public void periodic(){

    }
}