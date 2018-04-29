import ximureceiver, quaternion
import serial
ximu = ximureceiver.XimuReceiver()

# connecting x-IMU over USB
s=serial.Serial("/dev/ttyUSB0", 115200)

# connecting x-IMU over Bluetooth after Paired and Connected
#s=serial.Serial("/dev/rfcomm0", 115200)


# infinite loop for getting data
def main():
    while True:
        ximu.processNewChar(ord(s.read())%256)      # char to unsigned char conversion with modulus
    
        if ximu.isBattAndThermGetReady():
            BattAndTemp = ximu.getBattAndTherm()
            print "battery = ", BattAndTemp.battery
            print "tempereature = ", BattAndTemp.thermometer
            
        if ximu.isInertialAndMagGetReady():
            InertialAndMag = ximu.getInertialAndMag()
            print "gyrX = ", InertialAndMag.gyrX
            print "gyrY = ", InertialAndMag.gyrY
            print "gyrZ = ", InertialAndMag.gyrZ
            
            print "accX = ", InertialAndMag.accX
            print "accY = ", InertialAndMag.accY
            print "accZ = ", InertialAndMag.accZ
            
            print "magX = ", InertialAndMag.magX
            print "magY = ", InertialAndMag.magY
            print "magZ = ", InertialAndMag.magZ
    
        if ximu.isQuaternionGetReady():
            Quaternions = ximu.getQuaternion()
            Quaternion = quaternion.Quaternion(Quaternions.w, Quaternions.x, Quaternions.y, Quaternions.z)
            EulerAngles = Quaternion.getEulerAngles()
            print "roll = ", EulerAngles.roll
            print "pitch = ", EulerAngles.pitch
            print "yaw = ", EulerAngles.yaw

if __name__ == "__main__":
    main()