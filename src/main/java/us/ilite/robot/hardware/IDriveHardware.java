package us.ilite.robot.hardware;

public interface IDriveHardware extends IHardware {

    void configureMode(ECommonControlMode pControlMode);

    void setImu(IMU pImu);
    IMU getImu();

    double getLeftInches();
    double getRightInches();

    double getLeftVelInches();
    double getRightVelInches();

    double getLeftVelTicks();
    double getRightVelTicks();

    double getLeftTarget();
    double getRightTarget();

    double getLeftCurrent();
    double getRightCurrent();

    double getLeftVoltage();
    double getRightVoltage();

}
