#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
from scipy.signal import firwin
from scipy.signal import freqz
from scipy.signal import lfilter
from scipy.signal import butter
from scipy.signal import sosfilt
from pylab import *
import pylab as plt
import numpy as np
import spidev
from time import sleep
import RPi.GPIO as GPIO
import socket
import cmd
import time
import traceback
import select
import sys

# """Setup and test connection to ground control station"""
host = "192.168.0.101"#"172.20.10.6"#"127.0.0.1"
port = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setblocking(0)

try:
	sock.connect((host,port))
except:
	'''traceback.print_exc()'''

# """Uses the "spidev" package ('sudo pip3 install spidev')"""
# """check dtparam=spi=on --> in /boot/config.txt or (set via 'sudo raspi-config')"""

# """Set threshold for IED detection marker in ADC samples"""
detection_threshold = 0.12 # type: float # voltage level (V)

# """Instatiate Rasp Pi GPIO Pins for test purposes"""
#GPIO.setmode(GPIO.BCM)
#GPIO.setwarnings(False)
#GPIO.setup(18, GPIO.OUT)

# """Define the sampling rate for the analog to digital converter"""
sampling_rate = 100000.0  # 100kHz
num_samples = 1000
kHz = 1000.0 # kHz
# """Linear time for creating sin wave"""
time = arange(0,0.01,1.0/sampling_rate)

# """Create a Bandpass filter to filter multiplied_signal"""
numtaps = 64
low_cutoff = 26000  # 26kHz
high_cutoff = 35000  # 35kHz
nyq_rate = sampling_rate / 2
bpf = firwin(numtaps, [low_cutoff / nyq_rate, high_cutoff / nyq_rate], pass_zero=False)
w, h = freqz(bpf)

# Create a HPF Butterworth to filter multiplied_signal
butter_low_cutoff = 500 # 500Hz
hpf = butter(numtaps, butter_low_cutoff/nyq_rate, btype='highpass', analog=False, output='sos')

# """Plot bandpass filter response"""
#plt.subplot(221)
#plot(w / (2 * pi), 20 * log10(abs(h)))
#plt.title('FIR Bandpass Filter Response')
#plt.ylabel('Magnitude (dB)')
#plt.xlabel('Frequency Coefficient')

class Prompt(cmd.Cmd):
    def do_send(self, msg):
        print("Sending: <", msg, ">")
        sock.sendto(str.encode(msg), (host, port))

    def default(self, msg):
        sock.sendto(str.encode(msg), (host, port))
        # sock.send(str.encode(msg))
        print("Sent: <", msg, ">")

    def do_EOF(self, line):
        return True

class MCP3201(object):
    """
    Functions for reading the MCP3201 12-bit A/D converter using the SPI bus either in MSB- or LSB-mode
    """
    def __init__(self, SPI_BUS, CE_PIN):
        """
        initializes the device, takes SPI bus address (which is always 0 on newer Raspberry models)
        and sets the channel to either CE0 = 0 (GPIO pin BCM 8) or CE1 = 1 (GPIO pin BCM 7)
        """
        if SPI_BUS not in [0, 1]:
            raise ValueError('wrong SPI-bus: {0} setting (use 0 or 1)!'.format(SPI_BUS))
        if CE_PIN not in [0, 1]:
            raise ValueError('wrong CE-setting: {0} setting (use 0 for CE0 or 1 for CE1)!'.format(CE_PIN))
        self._spi = spidev.SpiDev()
        self._spi.open(SPI_BUS, CE_PIN)
        self._spi.max_speed_hz = 3900000
        pass

    def readADC_MSB(self):
        """
        Reads 2 bytes (byte_0 and byte_1) and converts the output code from the MSB-mode:
        byte_0 holds two ?? bits, the null bit, and the 5 MSB bits (B11-B07),
        byte_1 holds the remaning 7 MBS bits (B06-B00) and B01 from the LSB-mode, which has to be removed.
        """
        bytes_received = self._spi.xfer2([0x00, 0x00])

        MSB_1 = bytes_received[1]
        MSB_1 = MSB_1 >> 1  # shift right 1 bit to remove B01 from the LSB mode

        MSB_0 = bytes_received[0] & 0b00011111  # mask the 2 unknown bits and the null bit
        MSB_0 = MSB_0 << 7  # shift left 7 bits (i.e. the first MSB 5 bits of 12 bits)

        return MSB_0 + MSB_1

    def readADC_LSB(self):
        """
        Reads 4 bytes (byte_0 - byte_3) and converts the output code from LSB format mode:
        byte 1 holds B00 (shared by MSB- and LSB-modes) and B01,
        byte_2 holds the next 8 LSB bits (B03-B09), and
        byte 3, holds the remaining 2 LSB bits (B10-B11).
        """
        bytes_received = self._spi.xfer2([0x00, 0x00, 0x00, 0x00])

        LSB_0 = bytes_received[1] & 0b00000011  # mask the first 6 bits from the MSB mode
        LSB_0 = bin(LSB_0)[2:].zfill(2)  # converts to binary, cuts the "0b", include leading 0s

        LSB_1 = bytes_received[2]
        LSB_1 = bin(LSB_1)[2:].zfill(8)  # see above, include leading 0s (8 digits!)

        LSB_2 = bytes_received[3]
        LSB_2 = bin(LSB_2)[2:].zfill(8)
        LSB_2 = LSB_2[0:2]  # keep the first two digits

        LSB = LSB_0 + LSB_1 + LSB_2  # concatenate the three parts to the 12-digits string
        LSB = LSB[::-1]  # invert the resulting string
        return int(LSB, base=2)

    def convert_to_voltage(self, adc_output, VREF=3.3):
        """
        Calculates analogue voltage from the digital output code (ranging from 0-4095)
        VREF could be adjusted here (standard uses the 3V3 rail from the Rpi)
        """
        return float32(adc_output * (VREF / (2 ** 12 - 1)))


def multiplier(signal):
    # """Create frequencies to use for signal processing by sinusoidal multiplication"""
    f1 = 15000 # 15kHz
    f2 = 30000 # 30kHz
    # """Multiply the input signal from the ADC by a sin wave to get f1 + f2 and f1 - f2 and separate frequency components"""
    sinwave = [np.sin(2 * np.pi * f2 * x/sampling_rate) for x in range(num_samples)] # 30kHz sin wave
    #sinwave = [np.sin(2 * np.pi * f1 * x/sampling_rate) for x in range(num_samples)] # 15kHz sin wave
    multiplied_signal = np.multiply(signal, sinwave)
    return multiplied_signal

def kill_script()
    sys.exit()
    pass

def IED_reading_function():
    try:
        for i in range(array_size):
            # Take in the ADC Output value as a bit
            ADC_output_code = MCP3201.readADC_MSB()
            # Convert the bits from the ADC to a float32 voltage
            ADC_voltage = MCP3201.convert_to_voltage(ADC_output_code)

            # Print the ADC output code and MCP3201 sampled ADC Voltage
            # print("MCP3201 output code (MSB-mode): %d" % ADC_output_code)
            # print("MCP3201 voltage: %0.2f V" % ADC_voltage)

            # Set the value in the array(i) to the ADC_voltage received
            # array[i] = np.float32(ADC_voltage)
            array[i] = ADC_voltage
            # print(array)

        # """Multiply the noisy input signal by a 15kHz sin wave to separate frequency components (f1-f2/f1+f2)"""
        #multiplied_signal = multiplier(array)
        # """Do Not Multiply the array - Skip and set equal to sampled array for testing"""
        multiplied_signal = array

        # """Perform an fft of the mutliplied signal (for plotting purposes otherwise commented out)"""
        #fft_multiplied_signal = np.fft.rfft(multiplied_signal)
        # """Take absolute value of fft data or else the data is useless (complex)"""
        #fft_multiplied_signal = np.abs(fft_multiplied_signal)
        
        # """Filter the signal using the bandpass filter"""
        #filtered_signal = lfilter(bpf, 1, multiplied_signal)
        # """Filter the signal using the highpass butterworth filter"""
        #filtered_signal = sosfilt(hpf, multiplied_signal)
        # """Do Not filter the signal array - Skip and set equal to sampled array for testing"""
        filtered_signal = multiplied_signal

        # """Perform an fft of the filtered signal (for plotting purposes otherwise commented out)"""
        #fft_filtered_signal = np.fft.rfft(filtered_signal)
        # """Take absolute value of fft data or else the data is useless (complex)"""
        #fft_filtered_signal = np.abs(fft_filtered_signal)

        # """Read in the filtered signal and take the average and max of it for flagging a detection"""
        data_extracted = fft_filtered_signal[260:340]
        threshold_avg = sum(data_extracted) / len(data_extracted)
        threshold_max = data_extracted.max()

        #print threshold_avg
        print threshold_max
        #print data_extracted

        # """Use threshold_max as a threshold for detection against set value above detection_threshold"""
        if threshold_max >= detection_threshold:
            # """Light up an LED on the raspberry pi if a detection is found (for testing purposes only otherwise commented out)"""
            # """Replace this portion of code with Mavlink message transmission to pixhawk to send GCS GPS coordinates of detection"""
            #print "Detection: LED On"
            #GPIO.output(18, GPIO.HIGH)
            # """Send message to the ground control station for detection"""
            sock.sendto(str.encode(time.time()), (host, port))
            print "Detection: Sending Message"
        else:
            #print "No Detection: LED Off"
            #GPIO.output(18, GPIO.LOW)

    except (KeyboardInterrupt):
        print('\n', "Exit on Ctrl-C: Good bye!")

    except:
        print("Other error or exception occurred!")
        raise

    finally:
        #print(array)
        #plt.subplot(221)
        #plt.plot(time, array)
        #plt.plot(time, multiplied_signal)
        #plt.title('Multiplied Input Signal')
        #plt.ylabel('Magnitude (W)')
        #plt.xlabel('Time (s)')
        #plt.subplot(222)
        #plt.plot(20 * log10(fft_multiplied_signal))
        #plt.title('Multiplied Signal FFT')
        #plt.ylabel('Magnitude (dB)')
        #plt.xlabel('Frequency (kHz)')
        #plt.subplot(223)
        #plt.plot(time, filtered_signal)
        #plt.title('Filtered Input Signal')
        #plt.ylabel('Magnitude (W)')
        #plt.xlabel('Time (s)')
        #plt.xlim(0,.0025)
        #plt.subplot(224)
        #plt.plot(20*log10(data_extracted))
        #plt.plot(20 * log10(fft_filtered_signal))
        #plt.title('Filtered Input Signal FFT')
        #plt.ylabel('Magnitude (dB)')
        #plt.xlabel('Frequency (kHz)')
        #plt.show()
        print()
        sleep(0.5)

if __name__ == '__main__':
    SPI_bus = 0
    CE = 0
    MCP3201 = MCP3201(SPI_bus, CE)
    i = 0
    array_size = num_samples
    array = np.zeros((array_size,1), dtype=float32)
    while True:
        IED_reading_function()
