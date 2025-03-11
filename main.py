# адрес на шине I2C, по умолчанию, 0x5D
import time
from machine import I2C
from sensor_pack_2.bus_service import I2cAdapter
from lps3xmod import LPS3xST, lps3x_measured_values

def show_data(values: lps3x_measured_values):
    print(f"Ambient air temperature [\u2103]: {values.temperature}; Ambient air pressure [hPa]: {values.pressure}")

if __name__ == '__main__':
    i2c = I2C(id=1, freq=400_000)  # on Arduino Nano RP2040 Connect tested
    adapter = I2cAdapter(i2c)
    pressure_sensor = LPS3xST(adapter=adapter, address=0x5D)
    #
    print("LPS3x demo...")
    print(f"ID: 0x{pressure_sensor.get_id():X}")
    ds = pressure_sensor.get_data_status()
    print(f"Data status: {ds}")
    if ds.TempAvailable:
        print(f"Ambient air temperature [\u2103]: {pressure_sensor.get_measurement_value(0)}")
    # print(f"Ambient air pressure [hPa]: {pressure_sensor.get_measurement_value(1)}")

    _meas_vals = None
    single_shot_wait_time = 1000
    wait_func = time.sleep_ms
    repeat_cnt = 50
    print("\nРежим измерения по запросу (однократный)!")
    for i in range(repeat_cnt):
        pressure_sensor.start_measurement(continuous_mode=False, raw_odr=0)
        wait_func(single_shot_wait_time)
        ds = pressure_sensor.get_data_status()
        if ds.TempAvailable and ds.PressAvailable:
            _meas_vals = pressure_sensor.get_measurement_value(3)
        show_data(_meas_vals)

    print("\nРежим измерения периодический (автоматический)!")
    pressure_sensor.start_measurement(continuous_mode=True, raw_odr=1)
    delay_time = pressure_sensor.get_conversion_cycle_time()
    print(f"delay time [ms]: {delay_time}")
    wait_func(delay_time)
    for cnt, meas_values in enumerate(pressure_sensor):
        show_data(meas_values)
        if cnt > repeat_cnt:
            break
        wait_func(delay_time)