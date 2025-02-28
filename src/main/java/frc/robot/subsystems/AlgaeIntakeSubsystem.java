package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase{
    public SparkMax m_leader; 
    public SparkMax m_follower;

    public SparkMaxConfig leaderMotorConfig;
    public SparkMaxConfig followerMotorConfig;

    public AlgaeIntakeSubsystem(){
        m_leader = new SparkMax(ArmConstants.INTAKE_LEADER_ID, MotorType.kBrushless);
        m_follower = new SparkMax(ArmConstants.INTAKE_FOLLOWER_ID, MotorType.kBrushless);
        
        leaderMotorConfig = new SparkMaxConfig();
        leaderMotorConfig
        .inverted(false);
        m_leader.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        followerMotorConfig = new SparkMaxConfig();
        followerMotorConfig
        .inverted(true)
        .follow(ArmConstants.INTAKE_LEADER_ID);

        m_follower.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void changeSpeed(Double targetSpeed){
        m_leader.set(targetSpeed);
    }

    public Command forward(){
        return this.runOnce(() -> changeSpeed(ArmConstants.INTAKE_SPEED));
    }
    
    public Command reverse(){
        return this.runOnce(() -> changeSpeed(-ArmConstants.INTAKE_SPEED));
    }
    
    public Command zero(){
        return this.runOnce(() -> changeSpeed(0.0));
    }
}
