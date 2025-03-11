from collections import namedtuple

from sensor_pack_2 import bus_service
from sensor_pack_2.base_sensor import IDentifier, IBaseSensorEx, DeviceEx, Iterator, check_value
from sensor_pack_2.bitfield import bit_field_info
from sensor_pack_2.bitfield import BitFields

# serial_number_sht4x = namedtuple("serial_number_sht4x", "word_0 word_1")
# measured_values_sht4x = namedtuple("measured_values_sht4x", "T RH")
# Low-pass configuration
LPFP_config = namedtuple("LPFP_config", "enabled config")
# содержимое регистра состояние датчика
# TempOverrun - Temperature data overrun
# PressureOverrun - Pressure data overrun
# TempAvailable - Temperature data available
# PressAvailable - Pressure data available
lps3x_status = namedtuple("lps3x_status", "TempOverrun PressureOverrun TempAvailable PressAvailable")
lps3x_measured_values = namedtuple("lps3x_measured_values", "pressure temperature pressure_offset")

class LPS3xST(IDentifier, IBaseSensorEx, Iterator):
    """Класс для работы с MEMS датчиком давления от STMicroelectronics - LPS3x.
    Class for work with STMicroelectronics LPS3x MEMS pressure sensor"""

    _ctrl_reg_1 =   (bit_field_info(name='ODR', position=range(4, 7), valid_values=range(6), description=None),    # Output data rate selection
                    bit_field_info(name='EN_LPFP', position=range(3, 4), valid_values=None, description=None),  # Enable low-pass filter on pressure data
                    bit_field_info(name='LPFP_CFG', position=range(2, 3), valid_values=range(6), description=None), # Low-pass configuration register
                    # Бит BDU используется для запрета обновления выходных регистров между считыванием верхней и нижней частей регистра.
                    # В режиме по умолчанию (BDU = ‘0’) нижняя и верхняя части регистра обновляются непрерывно. Когда BDU активирован (BDU = ‘1’),
                    # содержимое выходных регистров не обновляется до тех пор, пока не будет считан PRESS_OUT_H (2Ah),
                    # чтобы избежать считывания значений, относящихся к разным образцам. Должен быть всегда в "1"
                    bit_field_info(name='BDU', position=range(1, 2), valid_values=None, description=None),  # Block data update
                    )
    _ctrl_reg_2 = (bit_field_info(name='BOOT', position=range(7, 8), valid_values=None, description=None),  # Reboot memory content
                   # должен быть всегда в "1"
                   bit_field_info(name='IF_ADD_INC', position=range(4, 5), valid_values=None, description=None), # Register address automatically incremented during a multiple byte access with a I²C
                   bit_field_info(name='SWRESET', position=range(2, 3), valid_values=None, description=None),   # Software reset
                   # Бит ONE_SHOT используется для начала нового преобразования, когда биты ODR[2:0] в CTRL_REG1 (10h) установлены в «000».
                   # Запись «1» в ONE_SHOT запускает единичное измерение давления и температуры. После завершения измерения бит ONE_SHOT автоматически очищается,
                   # новые данные становятся доступными в выходных регистрах, а биты STATUS (27h) обновляются.
                   bit_field_info(name='ONE_SHOT', position=range(1), valid_values=None, description=None), # One-shot enable
                   )

    def __init__(self, adapter: bus_service.BusAdapter, address=0x5D):
        """Если check_crc в Истина, то каждый, принятый от датчика пакет данных, проверяется на правильность путем
        расчета контрольной суммы."""
        check_value(address, range(0x5D, 0x5E), f"Неверный адрес устройства: {address}")
        self._connector = DeviceEx(adapter=adapter, address=address, big_byte_order=False)
        #
        self._single_shot_mode = None
        # self._continuously_mode = None
        # Частота обновления данных, Гц. Output data rate, Hz. 0..5.
        self._odr = None
        # Enable low-pass filter on pressure data
        self._lpfp_enabled = None
        # Low-pass filter configurations, 0..3
        self._lpfp_config = None
        # Block data update. (0: continuous update; # 1: output registers not updated until MSB and LSB have been read)
        self._bdu = None
        #
        # для удобства работы с настройками
        self._ctrl_reg_1_fields = BitFields(fields_info=LPS3xST._ctrl_reg_1)
        self._ctrl_reg_2_fields = BitFields(fields_info=LPS3xST._ctrl_reg_2)
        #
        self._buf_2 = bytearray(2)
        self._buf_3 = bytearray(3)
        # установка значений полей экземпляра класса
        self.raw_config_to_properties()
        self._init_ctrl_reg_2()

    def _read_buf_from_dev(self, addr: int, destination: bytearray) -> int:
        """Считывает из памяти датчика, начиная с адреса addr len(destination) байт в буфер destination.
        Возвращает длину буфера в байтах."""
        _len = len(destination)
        check_value(_len, range(2, 4), f"Неверная длина буфера [байт]: {_len}")
        _buf = self._buf_3 if 3 == _len else self._buf_2
        conn = self._connector
        conn.read_buf_from_mem(address=addr, buf=_buf, address_size=1)
        return _len

    def _get_temperature(self) -> float:
        """Возвращает температуру корпуса датчика в градусах Цельсия!"""
        buf = self._buf_2
        self._read_buf_from_dev(addr=0x2B, destination=buf)
        t = self._connector.unpack(fmt_char="h", source=buf)
        return 0.01 * t[0]

    def _get_pressure(self, press_out: bool = True) -> [int, float]:
        """Возвращает давление окружающего воздуха в hPa при press_out в Истина [float]!
        Возвращает 24-битные данные давления, вычитаемые из выходного сигнала датчика, измерение в режиме
        авто обнуления (auto-zero mode) при press_out в Ложь [int]!"""
        buf = self._buf_3
        offs = 0x28 if press_out else 0x15
        self._read_buf_from_dev(addr=offs, destination=buf)
        _p = buf[0] + (buf[1] << 8) + (buf[2] << 16)
        if not press_out:
            return _p
        return 0.00024414062 * _p

    def _read_reg_8bit(self, addr: int) -> int:
        """Возвращает содержимое восьмибитного регистра устройства с адресом addr"""
        return self._connector.read_reg(reg_addr=addr, bytes_count=1)[0]

    def _write_reg_8bit(self, addr: int, value: int):
        """Запиcывает в восьмибитный регистр устройства с адресом addr значение value"""
        self._connector.write_reg(reg_addr=addr, value=value, bytes_count=1)

    def _init_ctrl_reg_2(self, one_shot: bool = False, sw_reset: bool = False, default_value: int = 0x10):
        """Записывает в биты ctrl_reg_2 значения по умолчанию.
        Всегда включаю в 1 бит IF_ADD_INC!"""
        _addr = 0x11
        raw = self._read_reg_8bit(addr=_addr)
        if one_shot:
            raw |= 0x01
        if sw_reset:
            raw |= 0x04
        self._write_reg_8bit(addr=_addr, value=raw | default_value)

    @staticmethod
    def _hz_from_odr(raw_odr: int) -> int:
        """Возвращает частоту преобразования в Гц, соответствующую параметру raw_odr."""
        check_value(raw_odr, range(6), f"Неверное значение параметра raw_odr: {raw_odr}")
        _hz = 0, 1, 10, 25, 50, 75  # зависимость ODR в Гц от raw ODR.
        return _hz[raw_odr]

    def raw_config_to_properties(self):
        """Преобразует содержимое регистра управления 1 в свойства экземпляра класса."""
        bit_fields = self._ctrl_reg_1_fields
        bit_fields.source = self._read_reg_8bit(0x10)
        #
        self._odr = bit_fields['ODR']
        self._lpfp_enabled = bit_fields['EN_LPFP']
        self._lpfp_config = bit_fields['LPFP_CFG']
        self._bdu = bit_fields['BDU']

    def properties_to_raw_config(self) -> int:
        """Преобразует значения свойств экземпляра класса в число, которое должно быть записано в CTRL_REG1 (0x10)"""
        _cfg = self._read_reg_8bit(addr=0x10)
        bit_fields = self._ctrl_reg_1_fields
        bit_fields.source = _cfg
        #
        bit_fields['ODR'] = self.get_output_data_rate()
        bit_fields['EN_LPFP'] = self._lpfp_enabled
        bit_fields['LPFP_CFG'] = self._lpfp_config
        bit_fields['BDU'] = True    # 1: output registers not updated until MSB and LSB have been read)
        #
        return bit_fields.source

    # IDentifier
    def get_id(self) -> int:
        """Возвращает ответ на команду 'WHO_AM_I'"""
        return self._read_reg_8bit(0x0F)

    def soft_reset(self):
        """Програмный сброс устройства"""
        addr = 0x11
        raw = self._read_reg_8bit(addr)
        self._write_reg_8bit(addr, raw | 0x04)

    # IBaseSensorEx
    def get_data_status(self) -> lps3x_status:
        """Возвращает состояние готовности данных для считывания?"""
        stat = self._read_reg_8bit(addr=0x27)
        return lps3x_status(TempOverrun=bool(0x20 & stat), PressureOverrun=bool(0x10 & stat),
                            TempAvailable=bool(0x02 & stat), PressAvailable=bool(0x01 & stat))

    def get_measurement_value(self, value_index: [int] = 3) -> lps3x_measured_values:
        """Возвращает измеренное датчиком значение(значения) по его индексу/номеру."""
        if 1 == value_index:    # температура окружающего воздуха
            return lps3x_measured_values(pressure=None, temperature=self._get_temperature(), pressure_offset=None)
        if 2 == value_index:    # давление окружающего воздуха
            return lps3x_measured_values(pressure=self._get_pressure(press_out=True), temperature=None, pressure_offset=None)
        if 3 == value_index:    # давление и температура окружающего воздуха
            _press = self._get_pressure(press_out=True)
            _temp = self._get_temperature()
            return lps3x_measured_values(pressure=_press, temperature=_temp, pressure_offset=None)
        if 4 == value_index:    # pressure offset
            return lps3x_measured_values(pressure=None, temperature=None, pressure_offset=self._get_pressure(press_out=False))

    def get_conversion_cycle_time(self) -> int:
        """Возвращает время в мс преобразования сигнала в цифровой код и готовности его для чтения по шине!
        Для текущих настроек датчика. При изменении настроек следует заново вызвать этот метод!"""
        _odr = self.get_output_data_rate()
        if 0 == _odr:
            return 14 # [ms] 1/75  допустим(!) это минимальное время преобразования в однократном режиме (по запросу) измерения

        return int(1_000/LPS3xST._hz_from_odr(_odr))  # время в [мс]!!!

    def start_measurement(self, continuous_mode: bool = True, raw_odr: int = 1):
        """Настраивает параметры датчика и запускает процесс измерения.
        raw_odr - сырое значение output data rate 0..5, 0 - только для режима однократных измерений (измерение по запросу!). См. get_output_data_rate.
        continuous_mode в Истина - режим автоматических периодических измерений с 'частотой' raw_odr.
        continuous_mode в Ложь - режим измерений по запросу (однократные не периодические измерения)."""
        if continuous_mode and 0 == raw_odr:
            raise ValueError(f"В автоматическом режиме измерений значение raw_odr не должно быть равно нулю!")
        if not continuous_mode and 0 != raw_odr:
            raise ValueError(f"В режиме измерений по запросу значение raw_odr должно быть равно нулю!")
        _raw_odr = raw_odr if continuous_mode else 0
        self._odr = _raw_odr
        raw_cfg_1 = self.properties_to_raw_config() # для записи в ctrl_reg_1
        self._write_reg_8bit(addr=0x10, value=raw_cfg_1)
        # для запись в ctrl_reg_2
        self._init_ctrl_reg_2(not continuous_mode, sw_reset=False)
        # обновляю значения полей в соответствии с настройками датчика
        self.raw_config_to_properties()


    def is_single_shot_mode(self) -> bool:
        """Возвращает Истина, когда датчик находится в режиме однократных измерений,
        каждое из которых запускается методом start_measurement"""
        return 0 == self.get_output_data_rate()

    def is_continuously_mode(self) -> bool:
        """Возвращает Истина, когда датчик находится в режиме многократных измерений,
        производимых автоматически. Процесс запускается методом start_measurement"""
        return not self.is_single_shot_mode()

    def get_output_data_rate(self) -> int:
        """Return raw(!) output data rate (0..5):
        0 - Power down / one-shot mode enabled/0 Hz;
        1 - 1 Hz;
        2 - 10 Hz;
        3 - 25 Hz;
        4 - 50 Hz;
        5 - 75 Hz;"""
        return self._odr

    def get_lpfp_config(self) -> LPFP_config:
        """Возвращает Low-pass filter configurations."""
        return LPFP_config(enabled=self._lpfp_enabled, config=self._lpfp_config)

    # Iterator
    def __next__(self) -> [lps3x_measured_values, None]:
        if self.is_continuously_mode():
            # режим непрерывного преобразования!
            stat = self.get_data_status()
            if stat.PressAvailable and stat.TempAvailable:
                return self.get_measurement_value(3)
            if stat.PressAvailable and not stat.TempAvailable:
                return self.get_measurement_value(2)
            if not stat.PressAvailable and stat.TempAvailable:
                return self.get_measurement_value(1)
        return None