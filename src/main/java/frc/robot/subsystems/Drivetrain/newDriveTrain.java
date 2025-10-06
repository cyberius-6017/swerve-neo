package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class newDriveTrain extends SubsystemBase{

    private AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

    private newSwerveModule[] swerveModules;
    private SwerveModulePosition[] swervePositions;

    private Field2d field;
    private SwerveDriveOdometry swerveOdometry;
    private SwerveDrivePoseEstimator swervePoseEstimator;
    // private NetworkTable limef;
    // private NetworkTable limeb;
    


    public newDriveTrain(){

        navx.reset();
        zeroNavX();
        navx.setAngleAdjustment(90);

        field = new Field2d();
        SmartDashboard.putData("Field: ", field);
        
        
        
        swerveModules =
            new newSwerveModule[] {
                new newSwerveModule(0, Constants.Swerve.Mod0.constants),
                new newSwerveModule(1, Constants.Swerve.Mod1.constants),
                new newSwerveModule(2, Constants.Swerve.Mod2.constants),
                new newSwerveModule(3, Constants.Swerve.Mod3.constants),
            };

        swervePositions = 
            new SwerveModulePosition[]{
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
            };
            
        // swerveWheels =
        //     new SwerveDriveWheelPositions[]{
        //         new SwerveDriveWheelPositions(swervePositions)
        //     };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveOdoKinematics, 
                                                 getOdoYaw(), 
                                                 swervePositions); 
        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveOdoKinematics, 
                                                           getOdoYaw(), 
                                                           swervePositions, 
                                                           getPose());
        field.setRobotPose(getPose());
        // limef = NetworkTableInstance.getDefault().getTable(Constants.Sensors.limef);
        // limeb = NetworkTableInstance.getDefault().getTable(Constants.Sensors.limeb);  
    }

    public void drive(Translation2d translation, double rotation, boolean isFieldDrive, boolean isOpenloop){

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveOdoKinematics.toSwerveModuleStates(isFieldDrive ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), 
                                                                                                                        translation.getY(),
                                                                                                                        rotation, 
                                                                                                                        getOdoYaw())
                                                                                : new ChassisSpeeds(-translation.getY(),
                                                                                                    translation.getX(),
                                                                                                    rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(newSwerveModule module : swerveModules){

            module.setDesiredState(swerveModuleStates[module.moduleID], isOpenloop);

        }

    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
    
        for (newSwerveModule module : swerveModules) {

          module.setDesiredState(desiredStates[module.moduleID], false);

        }

      }

      public Rotation2d getYaw() {

        double angle = (Constants.Swerve.invNavX) ? 360 - navx.getAngle() //NavX is flipped
                                                  : navx.getAngle();

        return Rotation2d.fromDegrees(angle);

    }

    public Rotation2d getOdoYaw() {

        double angle = (!Constants.Swerve.invNavX) ? 360 - navx.getAngle() //NavX is flipped
                                                   : navx.getAngle();
        return Rotation2d.fromDegrees(angle);

    }

    public void zeroNavX() {

        System.out.println("NavX zeroed correctly");
        navx.zeroYaw();

    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (newSwerveModule module : swerveModules) {

          states[module.moduleID] = module.getState();

        }

        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (newSwerveModule module : swerveModules) {

          positions[module.moduleID] = module.getPosition();

        }

        return positions;
    }

    public Pose2d getPose() {

        return swerveOdometry.getPoseMeters();
        //return swervePoseEstimator.getEstimatedPosition();
    }


    public void resetPose(Pose2d pose) {
    
        swerveOdometry.resetPosition(getOdoYaw(), getPositions(), pose);
    
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        //TODO: Check open loop
        this.drive(new Translation2d(speeds.vyMetersPerSecond, -speeds.vxMetersPerSecond ),speeds.omegaRadiansPerSecond,false,true);
    
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        
        return Constants.Swerve.swerveOdoKinematics.toChassisSpeeds(getStates());

    }

    public void goToCoorditanes(double x, double y, Rotation2d rotation){
        
        double speedx = (x - swervePoseEstimator.getEstimatedPosition().getX()) * Constants.Swerve.alignKP;
        double speedy = (y - swervePoseEstimator.getEstimatedPosition().getY()) * Constants.Swerve.alignKP;
        Rotation2d speedw = Rotation2d.fromDegrees((rotation.getDegrees() - swervePoseEstimator.getEstimatedPosition().getRotation().getDegrees()) * Constants.Swerve.alignKP);
        drive(new Translation2d(speedx, -speedy), speedw.getDegrees(), true, true);

    }

    
    public void updateOdometry() {

        

        swervePoseEstimator.update(getOdoYaw(),
                                   getPositions());
        
        swerveOdometry.resetPosition(getOdoYaw(), getPositions(), swervePoseEstimator.getEstimatedPosition());
    }

    @Override
    public void periodic() {
        //swerveOdometry.update(getOdoYaw(), getPositions());
        //swervePoseEstimator.update(getOdoYaw(), getPositions());
        // //swervePoseEstimator.addVisionMeasurement(getPose(), 0);
        // limef = NetworkTableInstance.getDefault().getTable(Constants.Sensors.limef);
        // limeb = NetworkTableInstance.getDefault().getTable(Constants.Sensors.limeb);        
        
        
        SmartDashboard.putString("Odometry: ", swerveOdometry.getPoseMeters().getTranslation().toString());
        SmartDashboard.putNumber("NavX", getYaw().getDegrees());
        SmartDashboard.putNumber("NavX Odo", getOdoYaw().getDegrees());

        // if(limeb.getEntry("tv").getDouble(0) == 1){

        //     poseData = limeb.getEntry("botpos_wpiblue").getDoubleArray(poseData);
        //     visionMeasurement = new Pose2d(poseData[0], poseData[1], getOdoYaw());
        //     swervePoseEstimator.addVisionMeasurement(visionMeasurement, Timer.getFPGATimestamp());     

        // }


        updateOdometry();
        field.setRobotPose(swervePoseEstimator.getEstimatedPosition());


        double loggingModulesStates [] = {
            swerveModules[0].getState().angle.getDegrees(), swerveModules[0].getState().speedMetersPerSecond * 50,
            swerveModules[1].getState().angle.getDegrees(), swerveModules[1].getState().speedMetersPerSecond * 50,
            swerveModules[2].getState().angle.getDegrees(), swerveModules[2].getState().speedMetersPerSecond * 50,
            swerveModules[3].getState().angle.getDegrees(), swerveModules[3].getState().speedMetersPerSecond * 50   
        };

        SmartDashboard.putNumber("Angle FL: ", swerveModules[0].getState().angle.getDegrees());
        SmartDashboard.putNumber("Angle FR: ", swerveModules[1].getState().angle.getDegrees());
        SmartDashboard.putNumber("Angle RL: ", swerveModules[2].getState().angle.getDegrees());
        SmartDashboard.putNumber("Angle RR: ", swerveModules[3].getState().angle.getDegrees());

        SmartDashboard.putNumber("Angle EFL: ", swerveModules[0].getCANCoderAngle().getDegrees());
        SmartDashboard.putNumber("Angle EFR: ", swerveModules[1].getCANCoderAngle().getDegrees());
        SmartDashboard.putNumber("Angle ERL: ", swerveModules[2].getCANCoderAngle().getDegrees());
        SmartDashboard.putNumber("Angle ERR: ", swerveModules[3].getCANCoderAngle().getDegrees());



        SmartDashboard.putNumberArray("SwerveModuleStates", loggingModulesStates);

    }
    
}