package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    SparkMax m_motor;
    SparkMaxConfig config;
    SoftLimitConfig limit;
    ClosedLoopConfig closedLoop;
    SparkClosedLoopController closedLoopController;

    public ClimberSubsystem(){
        m_motor = new SparkMax(ClimberConstants.CLIMEBR_MOTOR_ID, MotorType.kBrushless);
        config = new SparkMaxConfig();
        limit = new SoftLimitConfig();
        closedLoop = new ClosedLoopConfig();
        closedLoopController = m_motor.getClosedLoopController();
        
        limit.forwardSoftLimit(ClimberConstants.FORWARD_LIMIT);
        limit.reverseSoftLimit(ClimberConstants.REVERSE_LIMIT);
        limit.forwardSoftLimitEnabled(true);
        limit.reverseSoftLimitEnabled(true);

        closedLoop.p(ClimberConstants.PID_CONSTANTS.kP);
        closedLoop.i(ClimberConstants.PID_CONSTANTS.kI);
        closedLoop.d(ClimberConstants.PID_CONSTANTS.kD);
        closedLoop.velocityFF(0.0);
        closedLoop.outputRange(-1.0, 1.0);

        config.apply(closedLoop);
        config.apply(limit);
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        setPosition(ClimberConstants.RESET_POS);
    }

    private void setPosition(double position) {
        closedLoopController.setReference(position, ControlType.kPosition);
    }

    private void setVelocity(double velocity) {
        closedLoopController.setReference(velocity, ControlType.kVelocity);
    }

    public Command freezeClimber(){
        return this.runOnce(() -> setVelocity(0));
    }
    public Command resetPosition(){
        return this.runOnce(() -> setPosition(ClimberConstants.RESET_POS));
    }
    public Command startClimberUp(){
        return this.runOnce(() -> setVelocity(ClimberConstants.CLIMBER_SPEED));
    }
    public Command startClimberDown(){
        return this.runOnce(() -> setVelocity(-ClimberConstants.CLIMBER_SPEED));
    }
}
