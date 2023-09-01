import wpilib
import wpilib.drive
import wpimath.controller
import ctre
import navx
import numpy as np

seconds = 0

c_ArmLiftTotalValue = 540

c_ArmExtendMaxValue = 100

class Robot(wpilib.TimedRobot):
    
    def __init__(self, period: seconds = 0.02) -> None:
        super().__init__(period)

        self.MotorFL = ctre.WPI_TalonSRX(1)
        self.MotorFR = ctre.WPI_TalonSRX(2)
        self.MotorBL = ctre.WPI_TalonSRX(3)
        self.MotorBR = ctre.WPI_TalonSRX(4)
        self.ArmLift = ctre.WPI_TalonSRX(5)
        self.ArmExtend = ctre.WPI_TalonSRX(6)

        self.frontDrive = wpilib.drive.DifferentialDrive(self.MotorFL, self.MotorFR)
        self.backDrive = wpilib.drive.DifferentialDrive(self.MotorBL, self.MotorBR)

        self.ArmExtendEncoder = wpilib.Encoder(0, 1)
        self.ArmExtendEncoder.setDistancePerPulse(540/4)    # 180 distance per 4 pulses

        self.ArmLiftEncoder = wpilib.Encoder(2, 3)
        self.ArmLiftEncoder.setDistancePerPulse(25/4)       # 25 distance per 4 pulses

        self.DriveStick = wpilib.Joystick(0)

        self.HelperStick = wpilib.XboxController(3)

        self.PIDArmLift = wpimath.controller.PIDController(1, 0, 1)
        self.PIDArmExtend = wpimath.controller.PIDController(1, 0, 1)

        self.gyro = navx.AHRS(wpilib._wpilib.SPI.Port(0))

        self.timer = wpilib.Timer()


        self.liftTargetAngle = 0
        self.extendTargetAngle = 0
        self.previousTime = 0


    def DeltaTime(self):
        return self.timer.get() - self.previousTime

    
    def getArmLiftEncoderValue(self):
        return float(np.clip(self.ArmLiftEncoder.get(), 0, c_ArmLiftTotalValue))
    
    
    def getArmExtendEncoderValue(self):
        return float(np.clip(self.ArmExtend.get(), 0, c_ArmExtendMaxValue))



    def teleopInit(self) -> None:
        self.previousTime = 0
        self.timer.reset()
        self.timer.start()
        self.ArmLiftEncoder.reset()


    def teleopPeriodic(self) -> None:
        t_zangle = self.DriveStick.getRawAxis(2)
        t_xangle = self.DriveStick.getRawAxis(0)
        t_finalangle = t_zangle/2 + t_xangle/2
        self.frontDrive.arcadeDrive(self.DriveStick.getRawAxis(1) * self.DriveStick.getRawAxis(3), t_finalangle, True)
        self.backDrive.arcadeDrive(self.DriveStick.getRawAxis(1) * self.DriveStick.getRawAxis(3), t_finalangle, True)

        self.liftTargetAngle = np.clip(
            self.HelperStick.getLeftY() * c_ArmLiftTotalValue * self.DeltaTime() + self.liftTargetAngle, 
            0, 
            c_ArmLiftTotalValue
        )
        t_ArmLiftMotorPower = self.PIDArmLift.calculate(self.getArmLiftEncoderValue(), self.liftTargetAngle)
        if t_ArmLiftMotorPower > 0:
            self.ArmLift.set(t_ArmLiftMotorPower)
        else:
            self.ArmLift.set(t_ArmLiftMotorPower/2)

        self.extendTargetAngle = np.clip(
            self.HelperStick.getRightY() * c_ArmExtendMaxValue * self.DeltaTime()/2 + self.extendTargetAngle,
            0,
            c_ArmExtendMaxValue
        )
        self.ArmExtend.set(self.PIDArmLift.calculate(self.getArmExtendEncoderValue(), self.extendTargetAngle))

        self.previousTime = self.timer.get()


if __name__ == "__main__":
    wpilib.run(Robot)