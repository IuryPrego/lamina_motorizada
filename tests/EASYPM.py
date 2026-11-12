from ctypes import c_uint32,byref,create_string_buffer,c_bool,c_char_p,c_int,c_int16,c_double
from TLPMX import TLPMX
from TLPMX import TLPM_DEFAULT_CHANNEL
from typing import Union

        

class PowerMeter():
    def __init__(self, wavelength:Union[int,float]):
        # Find connected power meter devices.
        tlPM = TLPMX()
        deviceCount = c_uint32()
        tlPM.findRsrc(byref(deviceCount))

        resourceName = create_string_buffer(1024)

        for i in range(0, deviceCount.value):
            tlPM.getRsrcName(c_int(i), resourceName)
            print("Resource name of device", i, ":", c_char_p(resourceName.raw).value)
        print("")
        tlPM.close()

        # Connect to last device.
        tlPM = TLPMX()
        tlPM.open(resourceName, c_bool(True), c_bool(True))

        message = create_string_buffer(1024)
        tlPM.getCalibrationMsg(message,TLPM_DEFAULT_CHANNEL)
        print("Connected to device", i)
        print("Last calibration date: ",c_char_p(message.raw).value)
        print("")

        # Set wavelength in nm.
        _wavelength = c_double(wavelength)
        tlPM.setWavelength(_wavelength,TLPM_DEFAULT_CHANNEL)

        # Enable auto-range mode.
        # 0 -> auto-range disabled
        # 1 -> auto-range enabled
        tlPM.setPowerAutoRange(c_int16(1),TLPM_DEFAULT_CHANNEL)

        # Set power unit to Watt.
        # 0 -> Watt
        # 1 -> dBm
        tlPM.setPowerUnit(c_int16(0),TLPM_DEFAULT_CHANNEL)

        self.tlpmx = tlPM

    def measure(self):
        power =  c_double()
        self.tlpmx.measPower(byref(power),TLPM_DEFAULT_CHANNEL)
        return power.value

    def close(self):
        self.tlpmx.close()

