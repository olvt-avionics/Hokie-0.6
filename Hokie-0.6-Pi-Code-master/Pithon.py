import serial
import time
import os.path

try:
    ser = serial.Serial('/dev/ttyACM0', 115200)

    filePath = '/Data/'
    fileName = 'FlightDatan0.csv'
    bootCount = 0

    fileName = fileName[:10] + str(bootCount) + fileName[-6:]
    while os.path.exists(filePath + fileName):
        bootCount = bootCount + 1
        fileName = fileName[:10] + str(bootCount) + fileName[-6:]
        
    time.sleep(5)
    print('Script started successfully')

    try:
        os.system('sudo cp -r /Data /media/usb') #Change copy to directory to reflect drive being used
    except:
        print('Copy Error')

    print('Boot Number:')
    print(bootCount)

    while 1:
        readLine = ser.readline().decode('utf-8', errors = 'ignore')
        listLine = readLine.rstrip().split(",")
        if readLine and len(listLine) == 56:
            listLine.append(str(time.time()))
            listLine.append('\n')
            completeLine = ','.join(map(str, listLine))
            #print(completeLine) #Prints data
            try:
                lineCount = int(int(listLine[0]) / 10000)
                fileName = fileName[:10] + str(bootCount) + 'n' + str(lineCount) + fileName[-4:]
                if not os.path.exists(filePath + fileName):
                    file = open(filePath + fileName, 'w', newline = '')
                    file.write('Count, Ard Timestamp, X1 Accel, Y1 Accel, Z1 Accel, X2 A'
                           'ccel, Y2 Accel, Z2 Accel, System1, Gyro1, Accel1, Mag1, '
                           'Temp1, W1 Quat, X1 Quat, Y1 Quat, Z1 Quat, X1 Accel, Y1 '
                           'Accel, Z1 Accel, X1 Mag, Y1 Mag, Z1 Mag,X1 Gyro, Y1 Gyro'
                           ', Z1 Gyro, X1 Grav, Y1 Grav, Z1 Grav, X1 Fusion, Y1 Fusi'
                           'on, Z1 Fusion, System2, Gyro2, Accel2, Mag2, Temp2, W2 Q'
                           'uat, X2 Quat, Y2 Quat, Z2 Quat, X2 Accel, Y2 Accel, Z2 A'
                           'ccel, X2 Mag, Y2 Mag, Z2 Mag, X2 Gyro, Y2 Gyro, Z2 Gyro,'
                           'X2 Grav, Y2 Grav, Z2 Grav, X2 Fusion, Y2 Fusion, Z2 Fusi'
                           'on, Pi Timestamp\n')
                    file.close()
                file = open(filePath + fileName, 'a', newline = '')
                file.write(completeLine)
                file.close()
            except:
                print('Serial Decode Error')
except:
    #os.system('sudo reboot')
