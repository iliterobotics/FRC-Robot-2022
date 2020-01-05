package us.ilite.robot.modules;

import us.ilite.common.config.Settings;
import us.ilite.robot.hardware.ECommonControlMode;
import us.ilite.robot.hardware.ECommonNeutralMode;

import java.util.Objects;
import static java.lang.Math.*;

public class DriveMessage {

  public static final DriveMessage kBrake = new DriveMessage()
          .demand(0,0)
          .neutral(ECommonNeutralMode.BRAKE);

  public static final DriveMessage kNeutral = new DriveMessage()
          .demand(0,0)
          .neutral(ECommonNeutralMode.COAST);

  /**
   * FALSE if this message is from Throttle + Turn
   * TRUE if this message is from Left + Right
   */
  private boolean mIsDirect = false;

  private double mTurn = 0;
  private double mThrottle = 0;
  private double mLeftOutput = 0;
  private double mRightOutput = 0;
  private ECommonControlMode mControlMode = ECommonControlMode.PERCENT_OUTPUT;
  private ECommonNeutralMode mNeutralMode = ECommonNeutralMode.BRAKE;

  public DriveMessage() {

  }

  /**
   * Returns the expected left side power. If isDirect() returns TRUE, then the units will be dependent upon what method
   * created the drive message. Additionally, the values may not match [ -1 < value < 1 ]
   *
   * If isDirect() returns FALSE and the values are outside [-1 < value < 1], then call normalize().
   * @return expected left-side output
   */
  public double getLeftOutput() {
    if(mIsDirect) {
      return mLeftOutput;
    } else {
      return mThrottle + mTurn;
    }
  }


  /**
   * Returns the expected right side power. If isDirect() returns TRUE, then the units will be dependent upon what method
   * created the drive message. Additionally, the values may not match [ -1 < value < 1 ]
   *
   * If isDirect() returns FALSE and the values are outside [-1 < value < 1], then call normalize().
   * @return expected right-side output
   */
  public double getRightOutput() {
    if(mIsDirect) {
      return mRightOutput;
    } else {
      return mThrottle - mTurn;
    }
  }

  public ECommonControlMode getMode() {
    return mControlMode;
  }

  public ECommonNeutralMode getNeutral() {
    return mNeutralMode;
  }

  /**
   Normalizes the drive inputs so the driver does not over-saturate the commands.  For example, if both turn + throttle
   are at their max ranges, then the robot cannot respond with more than 100% power to one side.

   This method will skip normalization if the DriveMessage was created directly from left/right demands.

   @return this DriveMessage object to support the builder pattern.
   */
  public DriveMessage normalize() {
    if(!mIsDirect) {
      // Credit to 2363, Triple Helix
      // https://github.com/TripleHelixProgramming/HelixUtilities

      //read in joystick values from OI
      //range [-1, 1]
      //find the maximum possible value of (throttle + turn)
      //along the vector that the arcade joystick is pointing
      double saturatedInput;
      double greaterInput = max(abs(mThrottle), abs(mTurn));
      //range [0, 1]
      double lesserInput = min(abs(mThrottle), abs(mTurn));
      //range [0, 1]
      if (greaterInput > 0.0) {
        saturatedInput = (lesserInput / greaterInput) + 1.0;
        //range [1, 2]
      } else {
        saturatedInput = 1.0;
      }
      //scale down the joystick input values
      //such that (throttle + turn) always has a range [-1, 1]
      mThrottle = mThrottle / saturatedInput;
      mTurn = mTurn / saturatedInput;
    }
    return this;
  }

  /**
   Implements the same scaling function as CheesyDrive, where turn is scaled by throttle.
   This *should* give us better performance at low speeds + the benefits of "clamped turn" drive.

   This method will skip normalization if the DriveMessage was created directly from left/right demands.

   @return this DriveMessage object to support the builder pattern.
   */
  public DriveMessage calculateCurvature() {
    if(!mIsDirect) {
      mTurn = Math.abs(mThrottle) * mTurn * Settings.Drive.kTurnSensitivity;
    }
    return this;
  }

  /**
   *
   * @param pTurn
   * @return this DriveMessage object to support the builder pattern.
   */
  public DriveMessage turn(double pTurn) {
    mTurn = pTurn;
    mIsDirect = false;
    return this;
  }

  /**
   *
   * @param pThrottle
   * @return this DriveMessage object to support the builder pattern.
   */
  public DriveMessage throttle(double pThrottle) {
    mThrottle = pThrottle;
    mIsDirect = false;
    return this;
  }

  /**
   *
   * @param pControlMode Overriding control mode
   * @return this DriveMessage object to support the builder pattern.
   */
  public DriveMessage mode(ECommonControlMode pControlMode) {
    mControlMode = pControlMode;
    return this;
  }

  /**
   * Override the left and right side demands. Nothing is assumed about units or values; i.e. these could be in
   * meters/second or % outputs, depending on what the calling method needs.
   * @param pLeftDemand Direct left-side demand
   * @param pRightDemand Direct right-side demand
   * @return this DriveMessage object to support the builder pattern.
   */
  public DriveMessage demand(double pLeftDemand, double pRightDemand) {
    this.mLeftOutput = pLeftDemand;
    this.mRightOutput = pRightDemand;
    mIsDirect = true;
    return this;
  }

  public DriveMessage neutral(ECommonNeutralMode pMode) {
    this.mNeutralMode = pMode;
    return this;
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    DriveMessage that = (DriveMessage) o;
    return
            this.mIsDirect == that.mIsDirect &&
            Double.compare(that.mLeftOutput, mLeftOutput) == 0 &&
            Double.compare(that.mRightOutput, mRightOutput) == 0 &&
            mControlMode == that.mControlMode &&
            mNeutralMode == that.mNeutralMode
    ;
  }

  @Override
  public int hashCode() {
    return Objects.hash(mIsDirect, mTurn, mThrottle, mLeftOutput, mRightOutput, mControlMode, mNeutralMode);
  }

}
