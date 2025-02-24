package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{

  /*
    * Allows for control of the elevator with two motors, one is the leader(left) and performs 
    * PID calculations while the other(right) is inverted and follows the leader motor(left)
    * https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master
  */

  public SparkMax m_arm;

  public int kArmMotorId = 50;

  public SparkMaxConfig armMotorConfig;

  public SparkClosedLoopController closedLoopController;
  public RelativeEncoder arm_encoder;
  
  private Double kPos0 = 65.0;
  private Double kPos1 = 40.0;

  public ArmSubsystem() {
    m_arm = new SparkMax(kArmMotorId, MotorType.kBrushless);

    closedLoopController = m_arm.getClosedLoopController();
    arm_encoder = m_arm.getEncoder();
    armMotorConfig = new SparkMaxConfig();
    armMotorConfig
        .inverted(false);
    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    armMotorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    armMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // Set PID values for position control. We don't need to pass a closed loop
    // slot, as it will default to slot 0.
    .p(0.04)
    .i(0)
    .d(0)
    .outputRange(-0.1, 0.1)
    // Set PID values for velocity control in slot 1
    .p(0.0001, ClosedLoopSlot.kSlot1)
    .i(0, ClosedLoopSlot.kSlot1)
    .d(0, ClosedLoopSlot.kSlot1)
    .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
    .outputRange(-0.15, 0.15, ClosedLoopSlot.kSlot1);

        /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    m_arm.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Arm Target Position", 0);
    SmartDashboard.setDefaultNumber("Arm Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Arm Control Mode", false);
    SmartDashboard.setDefaultBoolean("Arm Reset Encoder", false);
  }


  public void changePosition(Double targetPosition){
      closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public Command armPosition0(){
    return this.runOnce(() -> changePosition(kPos0));
  }
  public Command armPosition1(){
    return this.runOnce(() -> changePosition(kPos1));
  }
}