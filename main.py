import time
from machine import I2C, Pin
from sensor_pack_2.bus_service import I2cAdapter
from sensor_pack_2.bmp_common import SensorMode
from lps3xmod import Lps33

I2C_ID = 1
SCL_PIN = 7
SDA_PIN = 6
I2C_FREQ = 400_000
SENSOR_ADDR = 0x5D
ITERATIONS = 33


def print_sensor_data(t_c: float, p_pa: float) -> None:
    """Вывод температуры и давления в привычных единицах."""
    print(f"T={t_c:.2f}°C, P={p_pa:.2f} Pa")


if __name__ == '__main__':
    # Инициализация шины
    i2c = I2C(id=I2C_ID, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=I2C_FREQ)
    adapter = I2cAdapter(i2c)

    print("Инициализация LPS33...")
    ps = Lps33(adapter=adapter, address=SENSOR_ADDR)
    print(f"Chip ID: 0x{ps.get_id().chip_id:02X}")

    # Мягкий сброс для гарантии корректного состояния регистров
    ps.soft_reset()
    time.sleep_ms(50)

    # РЕЖИМ ОДНОКРАТНЫХ ИЗМЕРЕНИЙ (Forced)
    print("\nРежим однократных измерений (Forced)")
    ps.set_power_mode(SensorMode.FORCED)
    cycle_time = ps.get_conversion_cycle_time()

    for _ in range(ITERATIONS):
        ps.start_measurement()  # Триггер One-Shot
        time.sleep_ms(cycle_time + 5)  # жду завершения преобразования + запас

        if ps.is_data_ready():
            t = ps.get_temperature()
            p = ps.get_pressure()  # Возвращает Па
            print_sensor_data(t, p)
        else:
            print("Data NOT ready!")

    # РЕЖИМ НЕПРЕРЫВНЫХ ИЗМЕРЕНИЙ (Normal)
    print("\nРежим непрерывных измерений (Normal)")
    ps.set_power_mode(SensorMode.NORMAL)
    ps.set_sampling_period(1)   # 1 Hz
    ps.start_measurement()  # Коммит настроек в железо
    cycle_time = ps.get_conversion_cycle_time()
    print(f"Cycle time: {cycle_time} ms")

    count = 0
    # Итератор использует метод __next__(), который возвращает MeasuredParams или None
    for params in ps:
        if params is None:
            print("Data NOT ready!")
        else:
            print_sensor_data(params.temperature, params.pressure)

        if count >= ITERATIONS:
            break
        time.sleep_ms(cycle_time)
        count += 1
