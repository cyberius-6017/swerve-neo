package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.newDriveTrain;

public class newDriveTrainCommand extends Command{

    private newDriveTrain driveTrain;
    private Supplier<Double> translation, strafe, rotation, rTrigger, lTrigger;
    private Supplier<Boolean> changeDrive, resetNavX;
    private boolean isRobotCentric = false;

    
    private SlewRateLimiter translationLimit = new SlewRateLimiter(3.0);
    private SlewRateLimiter strafeLimit = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimit = new SlewRateLimiter(3.0);

    

    public newDriveTrainCommand(newDriveTrain m_DriveTrain, Supplier<Double> translation, Supplier<Double> strafe, Supplier<Double> rotation, Supplier<Boolean> changeDrive, Supplier<Boolean> resetNavX, Supplier<Double> rTrigger, Supplier<Double> lTrigger){

        this.driveTrain = m_DriveTrain;
        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
        this.changeDrive = changeDrive;
        this.resetNavX = resetNavX;
        this.rTrigger = rTrigger;
        this.lTrigger = lTrigger;
        isRobotCentric = true;

        addRequirements(m_DriveTrain);

    }
    
    @Override
    public void execute() {

        if(changeDrive.get()){
            
            isRobotCentric = !isRobotCentric;

        }    

        if(resetNavX.get()){

            driveTrain.zeroNavX();

        }    

        double translationVal = translationLimit.calculate(MathUtil.applyDeadband(translation.get(), 
                                                                                  Constants.Swerve.stickDeadband));
        double strafeVal = strafeLimit.calculate(MathUtil.applyDeadband(strafe.get(), 
                                                                        Constants.Swerve.stickDeadband)) + (rTrigger.get() - lTrigger.get());
        double rotationVal = rotationLimit.calculate(MathUtil.applyDeadband(rotation.get(), 
                                                                            Constants.Swerve.stickDeadband));
        SmartDashboard.putString("Drive Mode:",  (isRobotCentric ? "Driving Field Oriented"
                                                                     : "Driving Robot Oriented"));
        driveTrain.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), rotationVal * Constants.Swerve.maxAngularVelocity, isRobotCentric, true);
    }
}