package frc.robot.subsystems.Drivetrain;


// import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
    private CANcoder angleEncoder;

    private StaticBrake brake;
    private final SparkClosedLoopController steerController;
    private final SimpleMotorFeedforward feedforward =  new SimpleMotorFeedforward(Constants.Swerve.driveKS, 
                                                                                   Constants.Swerve.driveKV, 
                                                                                   Constants.Swerve.driveKA);    
    
    public newSwerveModule(int moduleID, SwerveModuleConstants moduleConstants){
        
        this.moduleID = moduleID;
        angleOffset = moduleConstants.angleOffset;
        encoderOffset = moduleConstants.encoderOffset;

        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        //configAngleEncoder();

        steerMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        steerMotorEncoder = steerMotor.getEncoder();
        steerController = steerMotor.getClosedLoopController();
        
        configSteerMotor();

        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        // brake = new StaticBrake();
        // driveMotor.setControl(brake);
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

    /* 
    private void configAngleEncoder() {

     
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1));
        angleEncoder.getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    }
    */

    private void configSteerMotor() {

        steerMotor.restoreFactoryDefaults();
        steerMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        steerMotor.setInverted(Constants.Swerve.angleInvert);
        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotorEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        steerController.setP(Constants.Swerve.angleKP);
        steerController.setI(Constants.Swerve.angleKI);
        steerController.setD(Constants.Swerve.angleKD);
        steerController.setFF(Constants.Swerve.angleKFF);
        steerMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        steerMotor.burnFlash();
        resetToAbsolute();

    }

    private void configDriveMotor() {

        //driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit));
        //driveMotor.setInverted(Constants.Swerve.driveInvert);
        
        driveMotor.setControl(brake);
        
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.Swerve.driveKP;
        slot0Configs.kI = Constants.Swerve.driveKI;
        slot0Configs.kD = Constants.Swerve.driveKD;

        driveMotor.getConfigurator().apply(slot0Configs);

        driveMotor.setPosition(0.0);
        
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {

            double power = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(power);
        
        } 
        else {

            driveMotor.setControl(new VelocityDutyCycle(desiredState.speedMetersPerSecond, 
                                                        desiredState.speedMetersPerSecond, 
                                                        true, 
                                                        feedforward.calculate(desiredState.speedMetersPerSecond), 
                                                        0, 
                                                        true, 
                                                        false, 
                                                        false));
              
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