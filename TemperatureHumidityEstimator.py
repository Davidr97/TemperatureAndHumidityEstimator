
import fcntl
import time
import numpy as np

import Adafruit_ADS1x15
import RPi.GPIO as GPIO

adc = Adafruit_ADS1x15.ADS1115(busnum=1)
GAIN = 1

FIRST_REGIME = 20
SECOND_REGIME = 21
THIRD_REGIME = 22
CLASSIFY_TEMPERATURE = 5
CLASSIFY_HUMIDITY = 6
ABNORMAL_TEMPERATURE = 7
ABNORMAL_HUMIDITY = 8
TEMPERATURE_EXCEEDING_LIMIT = 9
HUMIDITY_EXCEEDING_LIMIT = 10


class SHT21:
    """Class to read temperature and humidity from SHT21, much of class was 
    derived from:
    http://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/Humidity/Sensirion_Humidity_SHT21_Datasheet_V3.pdf
    and Martin Steppuhn's code from http://www.emsystech.de/raspi-sht21"""

    # control constants
    _SOFTRESET = 0xFE
    _I2C_ADDRESS = 0x40
    _TRIGGER_TEMPERATURE_NO_HOLD = 0xF3
    _TRIGGER_HUMIDITY_NO_HOLD = 0xF5
    _STATUS_BITS_MASK = 0xFFFC

    # From: /linux/i2c-dev.h
    I2C_SLAVE = 0x0703
    I2C_SLAVE_FORCE = 0x0706

    # datasheet (v4), page 9, table 7, thanks to Martin Milata
    # for suggesting the use of these better values
    # code copied from https://github.com/mmilata/growd
    _TEMPERATURE_WAIT_TIME = 0.086  # (datasheet: typ=66, max=85)
    _HUMIDITY_WAIT_TIME = 0.030  # (datasheet: typ=22, max=29)

    def __init__(self, device_number=1):
        """Opens the i2c device (assuming that the kernel modules have been
        loaded).  Note that this has only been tested on first revision
        raspberry pi where the device_number = 0, but it should work
        where device_number=1"""
        self.i2c = open('/dev/i2c-%s' % device_number, 'r+', 0)
        fcntl.ioctl(self.i2c, self.I2C_SLAVE, 0x40)
        self.i2c.write(chr(self._SOFTRESET))
        time.sleep(0.100)

    def read_temperature(self):
        """Reads the temperature from the sensor.  Not that this call blocks
        for ~86ms to allow the sensor to return the data"""
        self.i2c.write(chr(self._TRIGGER_TEMPERATURE_NO_HOLD))
        time.sleep(self._TEMPERATURE_WAIT_TIME)
        data = self.i2c.read(3)
        if self._calculate_checksum(data, 2) == ord(data[2]):
            return self._get_temperature_from_buffer(data)

    def read_humidity(self):
        """Reads the humidity from the sensor.  Not that this call blocks 
        for ~30ms to allow the sensor to return the data"""
        self.i2c.write(chr(self._TRIGGER_HUMIDITY_NO_HOLD))
        time.sleep(self._HUMIDITY_WAIT_TIME)
        data = self.i2c.read(3)
        if self._calculate_checksum(data, 2) == ord(data[2]):
            return self._get_humidity_from_buffer(data)

    def close(self):
        """Closes the i2c connection"""
        self.i2c.close()

    def __enter__(self):
        """used to enable python's with statement support"""
        return self

    def __exit__(self, type, value, traceback):
        """with support"""
        self.close()

    @staticmethod
    def _calculate_checksum(data, number_of_bytes):
        """5.7 CRC Checksum using the polynomial given in the datasheet"""
        # CRC
        POLYNOMIAL = 0x131  # //P(x)=x^8+x^5+x^4+1 = 100110001
        crc = 0
        # calculates 8-Bit checksum with given polynomial
        for byteCtr in range(number_of_bytes):
            crc ^= (ord(data[byteCtr]))
            for bit in range(8, 0, -1):
                if crc & 0x80:
                    crc = (crc << 1) ^ POLYNOMIAL
                else:
                    crc = (crc << 1)
        return crc

    @staticmethod
    def _get_temperature_from_buffer(data):
        """This function reads the first two bytes of data and
        returns the temperature in C by using the following function:
        T = =46.82 + (172.72 * (ST/2^16))
        where ST is the value from the sensor
        """
        unadjusted = (ord(data[0]) << 8) + ord(data[1])
        unadjusted &= SHT21._STATUS_BITS_MASK  # zero the status bits
        unadjusted *= 175.72
        unadjusted /= 1 << 16  # divide by 2^16
        unadjusted -= 46.85
        return unadjusted

    @staticmethod
    def _get_humidity_from_buffer(data):
        """This function reads the first two bytes of data and returns
        the relative humidity in percent by using the following function:
        RH = -6 + (125 * (SRH / 2 ^16))
        where SRH is the value read from the sensor
        """
        unadjusted = (ord(data[0]) << 8) + ord(data[1])
        unadjusted &= SHT21._STATUS_BITS_MASK  # zero the status bits
        unadjusted *= 125.0
        unadjusted /= 1 << 16  # divide by 2^16
        unadjusted -= 6
        return unadjusted


class Controller:
    def __init__(self):
        self.sensor = THSensor(100, 100, 1, 1)
        self.runningMode = 1
        self.mode = {1: [100, 1, 100, 1], 2: [0, 50, 0, 50], 3: [25, 25]}

    def processADCReading(self, r1, r2, r3, r4):
        if self.runningMode == 3:
            if self.mode[3][0] != r1:
                if (r1 >= self.sensor.minTemperature and r1 <= self.sensor.maxTemperature):
                    self.mode[3][0] = r1
                    self.sensor.temperatureLimitChanged(r1)
            if self.mode[3][1] != r3:
                if (r3 >= self.sensor.minHumidity and r3 <= self.sensor.maxHumidity):
                    self.mode[3][1] = r3
                    self.sensor.humidityLimitChanged(r3)
        else:
            m = self.runningMode
            if self.mode[m][0] != r1:
                if m == 1:
                    self.mode[m][0] = r1
                    self.sensor.temperatureSamplesChanged(r1)
                else:
                    if r1 < r2:
                        self.mode[m][0] = r1
                        self.sensor.minTemperatureChanged(r1)
            if self.mode[m][1] != r2:
                if m == 1:
                    self.mode[m][1] = r2
                    self.sensor.temperatureTimeChanged(r2)
                else:
                    if r1 < r2:
                        self.mode[m][1] = r2
                        self.sensor.maxTemperatureChanged(r2)
            if self.mode[m][2] != r3:
                if m == 1:
                    self.mode[m][2] = r3
                    self.sensor.humiditySamplesChanged(r3)
                else:
                    if r3 < r4:
                        self.mode[m][2] = r3
                        self.sensor.minHumidityChanged(r3)
            if self.mode[m][3] != r4:
                if m == 1:
                    self.mode[m][3] = r4
                    self.sensor.humidityTimeChanged(r4)
                else:
                    if r3 < r4:
                        self.mode[m][3] = r4
                        self.sensor.maxHumidityChanged(r4)

    def setRunningMode(self, mode):
        self.runningMode = mode

    def showInfo(self):
        print(self.runningMode)
        print(self.mode)


class THSensor:
    def __init__(self, tSamples, hSamples, tTime, hTime):
        self.temperatureSamples = tSamples
        self.humiditySamples = hSamples
        self.temperatureTime = tTime
        self.humidityTime = hTime
        self.minTemperature = 0
        self.maxTemperature = 50
        self.minHumidity = 0
        self.maxHumidity = 50
        self.minCold = 25
        self.minLow = 25
        self.currentTemperatureData = list()
        self.currentHumidityData = list()
        self.temperatureData = 200 * [0]
        self.humidityData = 200 * [0]
        self.countT = 0
        self.countH = 0
        self.shouldWarnTemperature = False
        self.shouldWarnHumidity = False

    def readTemperature(self, sample):
        if len(self.currentTemperatureData) >= self.temperatureSamples:
            self.currentTemperatureData[:] = []
        if self.countT >= 200:
            self.countT = 0
            self.shouldWarnTemperature = True
        self.currentTemperatureData.append(sample)
        self.temperatureData[self.countT] = sample
        self.countT = self.countT + 1

    def readHumidity(self, sample):
        if len(self.currentHumidityData) >= self.humiditySamples:
            self.currentHumidityData[:] = []
        if self.countH >= 200:
            self.countH = 0
            self.shouldWarnHumidity = True
        self.currentHumidityData.append(sample)
        self.humidityData[self.countH] = sample
        self.countH = self.countH + 1

    def classify(self, data, boundary, which):
        count = 0.0
        limit = len(data) / 2
        for sample in data:
            if sample <= boundary:
                count = count + 1
        if count > limit:
            if which == 1:
                print('Cold')
                GPIO.output(CLASSIFY_TEMPERATURE,False)
            else:
                print('Low')
                GPIO.output(CLASSIFY_HUMIDITY,False)
        elif count < limit:
            if which == 1:
                print('Warm')
                GPIO.output(CLASSIFY_TEMPERATURE,True)
            else:
                print('High')
                GPIO.output(CLASSIFY_HUMIDITY,True)
        else:
            print('tie')

    def classifyTemperature(self):
        if len(self.currentTemperatureData) == self.temperatureSamples:
            self.classify(self.currentTemperatureData, self.minCold, 1)

    def classifyHumidity(self):
        if len(self.currentHumidityData) == self.humiditySamples:
            self.classify(self.currentHumidityData, self.minLow, 2)

    def warn(self, data1, data2, which):
        t1 = np.mean(data1)
        t2 = np.mean(data2)
        if np.abs(t1 - t2) > 1:
            if which == 1:
                print('Temperature Warning')
                for i in range(10):
                    GPIO.output(ABNORMAL_TEMPERATURE,False)
                    time.sleep(1)
                    GPIO.output(ABNORMAL_TEMPERATURE,True)
                    time.sleep(1)
                GPIO.output(ABNORMAL_TEMPERATURE,False)
            else:
                print('Humidity Warning')
                for i in range(10):
                    GPIO.output(ABNORMAL_HUMIDITY,False)
                    time.sleep(1)
                    GPIO.output(ABNORMAL_HUMIDITY,True)
                    time.sleep(1)
                GPIO.output(ABNORMAL_HUMIDITY,False)

    def warnTemperature(self):
        if len(self.currentTemperatureData) == self.temperatureSamples:
            if self.shouldWarnTemperature:
                self.warn(self.currentTemperatureData, self.temperatureData, 1)

    def warnHumidity(self):
        if len(self.currentHumidityData) == self.humiditySamples:
            if self.shouldWarnHumidity:
                self.warn(self.currentHumidityData, self.humidityData, 2)

    def temperatureSamplesChanged(self, s):
        self.temperatureSamples = s
        self.currentTemperatureData.clear()

    def humiditySamplesChanged(self, s):
        self.humiditySamples = s
        self.currentHumidityData.clear()

    def temperatureTimeChanged(self, t):
        self.temperatureTime = t

    def humidityTimeChanged(self, t):
        self.humidityTime = t

    def minTemperatureChanged(self, t):
        self.minTemperature = t

    def maxTemperatureChanged(self, t):
        self.maxTemperature = t

    def minHumidityChanged(self, t):
        self.minHumidity = t

    def maxHumidityChanged(self, t):
        self.maxHumidity = t

    def temperatureLimitChanged(self, l):
        self.minCold = l

    def humidityLimitChanged(self, l):
        self.minLow = l

    def showInfo(self):
        print('The number of temperature samples is: ')
        print(self.temperatureSamples)
        print('The temperature time is: ')
        print(self.temperatureTime)
        print('The number of humidity samples is: ')
        print(self.humiditySamples)
        print('The humidity time is: ')
        print(self.humidityTime)
        print('The minimum temperature is: ')
        print(self.minTemperature)
        print('The maximum temperature is: ')
        print(self.maxTemperature)
        print('The minimum humidity is: ')
        print(self.minHumidity)
        print('The maximum humidity is: ')
        print(self.maxHumidity)
        print('The min Cold is: ')
        print(self.minCold)
        print('The min Low is: ')
        print(self.minLow)


if __name__ == "__main__":
    sht21 = SHT21()
    c = Controller()
    temperatureStartTime = time.time()
    humidityStartTime = time.time()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(FIRST_REGIME, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(SECOND_REGIME, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(THIRD_REGIME, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(CLASSIFY_TEMPERATURE,GPIO.OUT)
    GPIO.setup(CLASSIFY_HUMIDITY,GPIO.OUT)
    GPIO.setup(ABNORMAL_TEMPERATURE,GPIO.OUT)
    GPIO.setup(ABNORMAL_HUMIDITY,GPIO.OUT)
    GPIO.setup(TEMPERATURE_EXCEEDING_LIMIT,GPIO.OUT)
    GPIO.setup(HUMIDITY_EXCEEDING_LIMIT,GPIO.OUT)
    prevButtonState1 = True
    buttonState1 = True
    prevButtonState2 = True
    buttonState2 = True
    prevButtonState3 = True
    buttonState3 = True
    while 1:
        buttonState1 = GPIO.input(FIRST_REGIME)
        if prevButtonState1 != buttonState1:
            prevButtonState1 = buttonState1
            if buttonState1:
                c.setRunningMode(1)
        buttonState2 = GPIO.input(SECOND_REGIME)
        if prevButtonState2 != buttonState2:
            prevButtonState2 = buttonState2
            if buttonState2:
                c.setRunningMode(2)
        buttonState3 = GPIO.input(THIRD_REGIME)
        if prevButtonState3 != buttonState3:
            prevButtonState3 = buttonState3
            if buttonState3:
                c.setRunningMode(3)
        r1 = adc.read_adc(0, gain=GAIN, data_rate=32) * 0.125
        r2 = adc.read_adc(1, gain=GAIN, data_rate=32) * 0.125
        r3 = adc.read_adc(2, gain=GAIN, data_rate=32) * 0.125
        r4 = adc.read_adc(3, gain=GAIN, data_rate=32) * 0.125
        c.processADCReading(r1,r2,r3,r4)
        c.sensor.classifyTemperature()
        c.sensor.classifyHumidity()
        c.sensor.warnTemperature()
        c.sensor.warnHumidity()
        temperatureEndTime = time.time()
        if (temperatureEndTime - temperatureStartTime) >= c.sensor.temperatureTime:
            t = sht21.read_temperature()
            temperatureStartTime = time.time()
        humidityEndTime = time.time()
        if (humidityEndTime - humidityStartTime) >= c.sensor.humidityTime:
            h = sht21.read_humidity()
            humidityStartTime = time.time()
        if t:
            if t < c.sensor.minTemperature or t > c.sensor.maxTemperature:
                GPIO.output(TEMPERATURE_EXCEEDING_LIMIT,True)
                time.sleep(1)
                GPIO.output(TEMPERATURE_EXCEEDING_LIMIT,False)
            c.sensor.readTemperature(t)
        if h:
            if h < c.sensor.minHumidity or h > c.sensor.maxHumidity:
                GPIO.output(HUMIDITY_EXCEEDING_LIMIT,True)
                time.sleep(1)
                GPIO.output(HUMIDITY_EXCEEDING_LIMIT,False)
            c.sensor.readHumidity(h)




