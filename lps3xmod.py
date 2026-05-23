# micropython
# mail: kolbasilyvasily@yandex.ru
# MIT license
import micropython
from micropython import const
from sensor_pack_2.base_sensor import DeviceEx, check_value, Iterator
from sensor_pack_2.bmp_common import (IBaseAirPresSensor, SensorMode,
                                      MeasuredParams, SensorID, OversamplingCoeff)

#  Регистры LPS33K
_REG_WHO_AM_I = const(0x0F)
_REG_CTRL_REG1 = const(0x10)
_REG_CTRL_REG2 = const(0x11)
_REG_STATUS = const(0x27)
_REG_PRESS_OUT = const(0x28)  # 0x28(XL), 0x29(L), 0x2A(H)
_REG_TEMP_OUT = const(0x2B)  # 0x2B(L), 0x2C(H)

#  Постоянные
_WHO_AM_I_VAL = const(0xB1)
_SENS_PRESS = const(4096)  # LSB/hPa
_SENS_TEMP = const(100)  # LSB/°C
_HPA_TO_PA = const(100)
_ODR_HZ = (0, 1, 10, 25, 50, 75)

@micropython.native
def _to_signed(raw: int, bits: int) -> int:
    """Преобразует беззнаковое сырое значение в знаковое (дополнение до 2)."""
    return raw - (1 << bits) if raw >= (1 << (bits - 1)) else raw


class Lps33(IBaseAirPresSensor, Iterator):
    """Модуль для управления LPS33K от ST, (300-1200 hPa).
    Калибровка выполнена на заводе, коэффициенты не требуются."""


    def __init__(self, adapter, address: int = 0x5D):
        self._connection = DeviceEx(adapter=adapter, address=address, big_byte_order=False)
        self._buf_5 = bytearray(5)  # burst-read: 3 байта P + 2 байта T
        wr_reg = self._connection.write_reg
        # инит: ODR=000(PD), BDU=1, IF_ADD_INC=1
        wr_reg(_REG_CTRL_REG1, 0x04, 1)
        wr_reg(_REG_CTRL_REG2, 0x10, 1)
        #
        self._mode = SensorMode.SLEEP
        self._odr_raw = 0
        self._odr_hz = 0

    # == ЯДРО ИЗМЕРЕНИЙ ==
    @micropython.native
    def _read_raw_data(self) -> tuple[int, int]:
        self._connection.read_buf_from_mem(_REG_PRESS_OUT, self._buf_5, 1)
        raw_p = _to_signed(int.from_bytes(self._buf_5[0:3], 'little'), 24)
        raw_t = _to_signed(int.from_bytes(self._buf_5[3:5], 'little'), 16)
        return raw_p, raw_t

    def get_temperature(self) -> float:
        _, raw_t = self._read_raw_data()
        return raw_t / _SENS_TEMP

    def get_pressure(self) -> float:
        raw_p, _ = self._read_raw_data()
        return _HPA_TO_PA * (raw_p / _SENS_PRESS)

    def is_data_ready(self) -> bool:
        return bool(self._connection.read_reg(_REG_STATUS, 1)[0] & 0x03)

    # == ИНТЕРФЕЙСНЫЕ МЕТОДЫ ==
    def get_id(self) -> SensorID:
        """Возвращает идентификатор датчика."""
        return SensorID(chip_id=_WHO_AM_I_VAL, revision_id=None, spare1=None, spare2=None)

    def soft_reset(self) -> None:
        """Программный сброс датчика (SWRESET)."""
        # SWRESET = бит 2 в CTRL_REG2 (0x11). Само-сбрасывается.
        self._connection.write_reg(_REG_CTRL_REG2, 0x04, 1)
        # sleep_ms(10)  # Ожидание завершения сброса и загрузки калибровки

    def get_error(self) -> int:
        """Возвращает флаги ошибок (переполнение данных P/T)."""
        stat = self._connection.read_reg(_REG_STATUS, 1)[0]
        # Бит 3: P_OR, Бит 4: T_OR -> сдвигаем в 0..1
        return (stat >> 3) & 0x03

    def get_data_status(self, raw: bool = True):
        """Возвращает статус готовности и переполнения."""
        stat = self._connection.read_reg(_REG_STATUS, 1)[0]
        if raw:
            return stat
        return {
            "press_ready": bool(stat & 0x01),
            "temp_ready": bool(stat & 0x02),
            "press_overrun": bool(stat & 0x08),
            "temp_overrun": bool(stat & 0x10)
        }

    def __next__(self) -> MeasuredParams | None:
        """Итератор: возвращает MeasuredParams в Normal mode."""
        if not self.is_continuously_mode():
            return None
        if self.is_data_ready():
            return MeasuredParams(
                temperature=self.get_temperature(),
                pressure=self.get_pressure()
            )
        return None

    # == УПРАВЛЕНИЕ РЕЖИМАМИ ==
    def set_power_mode(self, value: int | None = None) -> int:
        """Устанавливает режим: 0=Sleep (Power Down), 1=Forced, 2=Normal(Continuous)."""
        if value is None:
            return self._mode

        conn = self._connection

        check_value(value, range(3), f"Invalid mode: {value}")
        ctrl1 = conn.read_reg(_REG_CTRL_REG1, 1)[0]
        ctrl1 &= ~0x70  # Очищаем биты ODR[2:0] (биты 6:4)

        if SensorMode.SLEEP == value:  # SLEEP
            ctrl1 |= 0x00
            self._odr_hz = 0
        elif SensorMode.NORMAL == value:  # NORMAL
            # Использую кэшированный ODR. Если он 0, использую безопасное значение (25 Гц - индекс 3)
            odr_to_set = self._odr_raw if self._odr_raw != 0 else 3
            ctrl1 |= (odr_to_set << 4)
            self._odr_raw = odr_to_set
            self._odr_hz = _ODR_HZ[odr_to_set]

        conn.write_reg(_REG_CTRL_REG1, ctrl1, 1)
        self._mode = value
        return value

    def start_measurement(self):
        """Запуск однократного измерения (One-shot). Работает только при ODR=000."""
        if self._mode == SensorMode.NORMAL:
            return  # В непрерывном режиме датчик мерит сам, вызов не нужен
        #
        conn = self._connection
        # Forced mode требует ODR=000
        ctrl1 = conn.read_reg(_REG_CTRL_REG1, 1)[0]
        ctrl1 &= ~0x70
        conn.write_reg(_REG_CTRL_REG1, ctrl1, 1)

        # Триггер ONE_SHOT (бит 0 в CTRL_REG2)
        ctrl2 = conn.read_reg(_REG_CTRL_REG2, 1)[0]
        conn.write_reg(_REG_CTRL_REG2, ctrl2 | 0x01, 1)

    def get_conversion_cycle_time(self) -> int:
        """Возвращает время цикла в мс."""
        if SensorMode.FORCED == self._mode:
            return 20  # One-shot
        if self._odr_hz > 0:
            return int(1000 / self._odr_hz) + 10
        return 40

    def is_single_shot_mode(self) -> bool:
        return SensorMode.FORCED == self._mode

    def is_continuously_mode(self) -> bool:
        return SensorMode.NORMAL == self._mode

    # == ЗАГЛУШКИ ДЛЯ КОНТРАКТА ==
    def set_oversampling(self, temp: int | None = None, press: int | None = None)-> None | OversamplingCoeff:
        """LPS33K не поддерживает настройку oversampling. Возвращает фиксированные значения."""
        return OversamplingCoeff(temperature=0, pressure=0)

    def set_iir_filter(self, temp: int | None = None, press: int | None = None):
        """LPS33K использует общий LPFP. Настройка через CTRL_REG1[3:2] опциональна."""
        return 0, 0

    def set_sampling_period(self, value: int | None = None) -> int:
        """Устанавливает или возвращает индекс ODR (0..5).
        0=Power Down, 1=1Hz, 2=10Hz, 3=25Hz, 4=50Hz, 5=75Hz."""
        conn = self._connection
        if value is not None:
            check_value(value, range(6), f"Invalid ODR index: {value}")
            self._odr_raw = value
            self._odr_hz = _ODR_HZ[value]
            ctrl1 = conn.read_reg(_REG_CTRL_REG1, 1)[0]
            ctrl1 = (ctrl1 & ~0x70) | (value << 4)
            conn.write_reg(_REG_CTRL_REG1, ctrl1, 1)

        self._odr_raw = (conn.read_reg(_REG_CTRL_REG1, 1)[0] >> 4) & 0x07
        self._odr_hz = _ODR_HZ[self._odr_raw]
        return self._odr_raw