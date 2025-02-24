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

public class ElevatorSubsystem extends SubsystemBase{

  /*
    * Allows for control of the elevator with two motors, one is the leader(left) and performs 
    * PID calculations while the other(right) is inverted and follows the leader motor(left)
    * https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master
  */

  public SparkMax m_left; // leader
  public SparkMax m_right; // follower

  public int kLeftElevatorId = 11;
  public int kRightElevatorId = 12;

  public SparkMaxConfig leftMotorConfig;
  public SparkMaxConfig rightMotorConfig;

  public SparkClosedLoopController closedLoopController;
  public RelativeEncoder left_encoder;
  
  private Double kPos0 = 0.0;
  private Double kPos1 = 5.0;
  private Double kPos2 = 10.0;
  private Double kPos3 = 15.0;

  public ElevatorSubsystem() {
    m_left = new SparkMax(kLeftElevatorId, MotorType.kBrushless);
    m_right = new SparkMax(kRightElevatorId, MotorType.kBrushless);

    closedLoopController = m_left.getClosedLoopController();
    left_encoder = m_left.getEncoder();
    leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig
        .inverted(true);
    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    leftMotorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    leftMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // Set PID values for position control. We don't need to pass a closed loop
    // slot, as it will default to slot 0.
    .p(0.015)
    .i(0)
    .d(0)
    .outputRange(-1, 1)
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
    m_left.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize and configure the right/follower motor
    rightMotorConfig = new SparkMaxConfig();
    rightMotorConfig
        .inverted(false)
        .follow(m_left);
    m_right.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
  }


  public void changePosition(Double targetPosition){
      closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public Command elevatorPos0(){
    return this.runOnce(() -> changePosition(kPos0));
  }
  public Command elevatorPos1(){
    return this.runOnce(() -> changePosition(kPos1));
  }
  public Command elevatorPos2(){
    return this.runOnce(() -> changePosition(kPos2));
  }
  public Command elevatorPos3(){
    return this.runOnce(() -> changePosition(kPos3));
  }

  //@Override 
  // public void periodic() {
  //   if (SmartDashboard.getBoolean("Control Mode", false)) {
  //     /*
  //      * Get the target velocity from SmartDashboard and set it as the setpoint
  //      * for the closed loop controller.
  //      */
  //     double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
  //     closedLoopController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  //   } else {
  //     /*
  //      * Get the target position from SmartDashboard and set it as the setpoint
  //      * for the closed loop controller.
  //      */
  //     double targetPosition = SmartDashboard.getNumber("Target Position", 0);
  //     closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  //     System.out.println("position: " + targetPosition);
  //   }

  //   // Display encoder position and velocity
  //   SmartDashboard.putNumber("Actual Position", left_encoder.getPosition());
  //   SmartDashboard.putNumber("Actual Velocity", left_encoder.getVelocity());

  //   if (SmartDashboard.getBoolean("Reset Encoder", false)) {
  //       SmartDashboard.putBoolean("Reset Encoder", false);
  //       // Reset the encoder position to 0
  //       left_encoder.setPosition(0);
  //   }
  // }
}