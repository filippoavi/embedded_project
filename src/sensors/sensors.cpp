/* Reference used in this code:
 * https://github.com/arduino-libraries/Arduino_BHY2/tree/main
 */

#include "sensors.h"
#include "SensorTypes.h"

//------------------------------------------------------------------------------
//SPI sensors_spi(SPI_MOSI, SPI_MISO, SPI_SCK /*, SPI_PSELSS0 */);
enum bhy2_intf intf;
const int8_t sensorCS = D6;
static float dataBuffer[3][6]; // 3 axis, 6 sensors
struct bhy2_dev _bhy2;
CircularBuffer<SensorDataPacket, SENSOR_QUEUE_SIZE, uint8_t> _sensorQueue;
CircularBuffer<SensorLongDataPacket, LONG_SENSOR_QUEUE_SIZE, uint8_t> _longSensorQueue;

SensorXYZ accel_s(SENSOR_ID_ACC);
SensorXYZ gyro_s(SENSOR_ID_GYRO);
SensorXYZ mag_s(SENSOR_ID_MAG);
Sensor temperature_s(SENSOR_ID_TEMP);
Sensor pressure_s(SENSOR_ID_BARO);
SensorBSEC bsec_s(SENSOR_ID_BSEC);
//------------------------------------------------------------------------------
void sensorSetup() {
// BHY2 definitions
  uint8_t _workBuffer[WORK_BUFFER_SIZE];
  intf = BHY2_SPI_INTERFACE;
  int8_t result;

	Serial.print("Configuring SPI interface... ");
    
  // Configure SPI interface
  setup_interfaces(true, intf);

	Serial.println("SPI interface configured.");

  // BHY2 device initialization
  Serial.print("Initializing BHY2 device... ");
  result = bhy2_init(BHY2_SPI_INTERFACE, bhy2_spi_read, bhy2_spi_write, bhy2_delay_us, BHY2_RD_WR_LEN, NULL, &_bhy2);
  if (result != BHY2_OK)
  {
    printf("Error during initialization: %s\n", get_api_error(result));
  }
  Serial.println("BHY2 device initialized.");

	Serial.println("Boot BHY2 device... ");
	bhy2_soft_reset(&_bhy2);
	// BHY2 status
	uint8_t stat = 0;

	auto ret = bhy2_get_boot_status(&stat, &_bhy2);
	Serial.println(get_api_error(ret));
	Serial.println("Boot status: ");
	Serial.println(stat, HEX);

	ret = bhy2_boot_from_flash(&_bhy2);
	Serial.println(get_api_error(ret));

	ret = bhy2_get_boot_status(&stat, &_bhy2);
	Serial.println(get_api_error(ret));
	Serial.println("Boot status: ");
	Serial.println(stat, HEX);

	ret = bhy2_get_host_interrupt_ctrl(&stat, &_bhy2);
	Serial.println(get_api_error(ret));
	Serial.println("Interrupt control: ");
	Serial.println(stat, HEX);

	ret = bhy2_get_host_intf_ctrl(&stat, &_bhy2);
	Serial.println(get_api_error(ret));
	Serial.println("Interrupt control: ");
	Serial.println(stat, HEX);
	Serial.println("BHY2 device booted.");

	// Register system callbacks
  Serial.print("Registering system callbacks... ");
	bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parseMetaEvent, NULL, &_bhy2);
  bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parseMetaEvent, NULL, &_bhy2);
  bhy2_register_fifo_parse_callback(BHY2_SYS_ID_DEBUG_MSG, parseDebugMessage, NULL, &_bhy2);
  Serial.println("System callbacks registered.");

  ret = bhy2_get_and_process_fifo(_workBuffer, WORK_BUFFER_SIZE, &_bhy2);
	Serial.println(get_api_error(ret));

	// Register sensor callbacks --- DA QUI RIVEDERE!!!
  // All sensors' data are handled in the same generic way
  Serial.print("Registering sensor callbacks... ");
  for (uint8_t i = 1; i < BHY2_SENSOR_ID_MAX; i++) {
    bhy2_register_fifo_parse_callback(i, parseData, NULL, &_bhy2);
  }

  bhy2_update_virtual_sensor_list(&_bhy2);
  bhy2_get_virt_sensor_list(&_bhy2);
  Serial.println("Sensor callbacks registered.");

/*   // FROM HERE DIFFERENT - Sensor callback initialization
  bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC, sensor_data_callback, NULL, &_bhy2);
  bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GYRO, sensor_data_callback, NULL, &_bhy2);
  bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_MAG, sensor_data_callback, NULL, &_bhy2);
  bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_TEMP, sensor_data_callback, NULL, &_bhy2);
  bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_HUM, sensor_data_callback, NULL, &_bhy2);
  bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_BARO, sensor_data_callback, NULL, &_bhy2); */
  
  // Sensors begin
  //configure(100.0, 0);

  // Sensor configuration for each sensor
  accel_s.begin(100, 0);
  gyro_s.begin(100, 0);
  mag_s.begin(10, 0);
  temperature_s.begin(10, 0);
  pressure_s.begin(10, 0);
  bsec_s.begin(10, 0);
  
}
//------------------------------------------------------------------------------

// Define a macro function to generate functions to read sensor values to String
#define SENSOR_READ_FUNC(name, sens, axis) \
String name() { \
  return String(sens.axis()); \
}
// Functions to read sensor values as String
SENSOR_READ_FUNC(sensorReadAccX, accel_s, x);
SENSOR_READ_FUNC(sensorReadAccY, accel_s, y);
SENSOR_READ_FUNC(sensorReadAccZ, accel_s, z);
SENSOR_READ_FUNC(sensorReadGyroX, gyro_s, x);
SENSOR_READ_FUNC(sensorReadGyroY, gyro_s, y);
SENSOR_READ_FUNC(sensorReadGyroZ, gyro_s, z);
SENSOR_READ_FUNC(sensorReadMagX, mag_s, x);
SENSOR_READ_FUNC(sensorReadMagY, mag_s, y);
SENSOR_READ_FUNC(sensorReadMagZ, mag_s, z);
SENSOR_READ_FUNC(sensorReadTemperature, temperature_s, value);
SENSOR_READ_FUNC(sensorReadPressure, pressure_s, value);
SENSOR_READ_FUNC(sensorReadVOC, bsec_s, b_voc_eq);
SENSOR_READ_FUNC(sensorReadCO2, bsec_s, co2_eq);
SENSOR_READ_FUNC(sensorReadHumidity, bsec_s, comp_h);
//----------------------------------------------------------------------------
// Define a macro function to generate functions to get sensor values
#define SENSOR_GET_FUNC(name, sens, axis) \
float name() { \
  return float(sens.axis()); \
}
// Functions to get sensor values as float
SENSOR_GET_FUNC(sensorGetAccX, accel_s, x);
SENSOR_GET_FUNC(sensorGetAccY, accel_s, y);
SENSOR_GET_FUNC(sensorGetAccZ, accel_s, z);
SENSOR_GET_FUNC(sensorGetGyroX, gyro_s, x);
SENSOR_GET_FUNC(sensorGetGyroY, gyro_s, y);
SENSOR_GET_FUNC(sensorGetGyroZ, gyro_s, z);
SENSOR_GET_FUNC(sensorGetMagX, mag_s, x);
SENSOR_GET_FUNC(sensorGetMagY, mag_s, y);
SENSOR_GET_FUNC(sensorGetMagZ, mag_s, z);
SENSOR_GET_FUNC(sensorGetTemperature, temperature_s, value);
SENSOR_GET_FUNC(sensorGetPressure, pressure_s, value);
SENSOR_GET_FUNC(sensorGetVOC, bsec_s, b_voc_eq);
SENSOR_GET_FUNC(sensorGetCO2, bsec_s, co2_eq);
SENSOR_GET_FUNC(sensorGetHumidity, bsec_s, comp_h);
//----------------------------------------------------------------------------
// Callback per la gestione dei dati FIFO del sensore
void sensor_data_callback(const struct bhy2_fifo_parse_data_info *info, void *private_data)
{
    const uint8_t *data = info->data_ptr;

    switch (info->sensor_id)
    {
        case BHY2_SENSOR_ID_ACC: // Accelerometro
        {
            struct bhy2_data_xyz accel;
            bhy2_parse_xyz(data, &accel);
            printf("Accelerazione (m/s^2): X=%.3f, Y=%.3f, Z=%.3f\n",
                   accel.x / 4096.0f, accel.y / 4096.0f, accel.z / 4096.0f);
            // Save the data in the buffer
            dataBuffer[0][0] = accel.x / 4096.0f; // X
            dataBuffer[1][0] = accel.y / 4096.0f; // Y
            dataBuffer[2][0] = accel.z / 4096.0f; // Z
            break;
        }
        case BHY2_SENSOR_ID_GYRO: // Giroscopio
        {
            struct bhy2_data_xyz gyro;
            bhy2_parse_xyz(data, &gyro);
            printf("Giroscopio (deg/s): X=%.3f, Y=%.3f, Z=%.3f\n",
                   gyro.x * 2000.0f / 32768.0f, gyro.y * 2000.0f / 32768.0f, gyro.z * 2000.0f / 32768.0f);
            // Save the data in the buffer
            dataBuffer[0][1] = gyro.x * 2000.0f / 32768.0f; // X
            dataBuffer[1][1] = gyro.y * 2000.0f / 32768.0f; // Y
            dataBuffer[2][1] = gyro.z * 2000.0f / 32768.0f; // Z
            break;
        }
        case BHY2_SENSOR_ID_MAG: // Magnetometro
        {
            struct bhy2_data_xyz mag;
            bhy2_parse_xyz(data, &mag);
            printf("Campo magnetico (uT): X=%.3f, Y=%.3f, Z=%.3f\n",
                   mag.x * 2500.0f / 32768.0f, mag.y * 2500.0f / 32768.0f, mag.z * 2500.0f / 32768.0f);
            // Save the data in the buffer
            dataBuffer[0][2] = mag.x * 2500.0f / 32768.0f; // X
            dataBuffer[1][2] = mag.y * 2500.0f / 32768.0f; // Y
            dataBuffer[2][2] = mag.z * 2500.0f / 32768.0f; // Z
            break;
        }
        case BHY2_SENSOR_ID_TEMP: // Temperatura
        {
            float temperature;
            bhy2_parse_temperature_celsius(data, &temperature);
            printf("Temperatura (°C): %.2f\n", temperature);
            // Save the data in the buffer
            dataBuffer[0][3] = temperature; // Temperature
            break;
        }
        case BHY2_SENSOR_ID_HUM: // Umidità
        {
            float humidity;
            bhy2_parse_humidity(data, &humidity);
            printf("Umidità relativa (%%): %.2f\n", humidity);
            // Save the data in the buffer
            dataBuffer[0][4] = humidity; // Humidity
            break;
        }
        case BHY2_SENSOR_ID_BARO: // Pressione atmosferica
        {
            float pressure;
            bhy2_parse_pressure(data, &pressure);
            printf("Pressione atmosferica (Pa): %.2f\n", pressure);
            // Save the data in the buffer
            dataBuffer[0][5] = pressure; // Pressure
            break;
        }
        default:
            break;
    }
}
//----------------------------------------------------------------------------
// Function to read the FIFOs and parse the data in the BMI260AP
int8_t sensorUpdate() {
  // Probabily add update from Nicla!!!
  
  int8_t result = BHY2_OK;
  result = bhy2_get_and_process_fifo(NULL, 0, &_bhy2);
        if (result != BHY2_OK)
        {
            printf("Errore FIFO: %s\n", get_api_error(result));
        }
  return result;
}
void dataBufferPrint() {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++) {
      Serial.print("i: " + String(i) + " - j: " + String(j) + " - ");
      Serial.print(dataBuffer[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
void spiTest() {
    uint8_t reg_addr = 0X2B; // Example adderss to read from
    uint8_t reg_data[1]; // Buffer to store the read data
    uint32_t length = 1; // Number of bytes to read
    int8_t result = bhy2_get_regs(reg_addr, reg_data, length, &_bhy2);
    if (result != BHY2_OK) {
        Serial.print("Error reading register: ");
        Serial.println(get_api_error(result));
        return;
    }

    // Prints the read data
    Serial.print("Read data from register 0x");
    Serial.print(reg_addr);
    Serial.print(": 0x");
    Serial.println(reg_data[0]);
    
    //int8_t result = bhy2_spi_read(reg_addr, reg_data, length, NULL);
}
//----------------------------------------------------------------------------
void parseMetaEvent(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
  uint8_t meta_event_type = callback_info->data_ptr[0];
  uint8_t byte1 = callback_info->data_ptr[1];
  uint8_t byte2 = callback_info->data_ptr[2];
  uint32_t s, ns;
  const char *event_text;

  if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
  {
    event_text = "[META EVENT]";
  }
  else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
  {
    event_text = "[META EVENT WAKE UP]";
  }
  else
  {
    return;
  }

  convertTime(*callback_info->time_stamp, &s, &ns);

  struct parse_ref *parse_table = (struct parse_ref*)callback_ref;
  (void)parse_table;
  switch (meta_event_type)
  {
    case BHY2_META_EVENT_FLUSH_COMPLETE:
      //printf("%s; T: %u.%09u; Flush complete for sensor id %u\r\n", event_text, s, ns,byte1);
      Serial.print(event_text);
      Serial.print(" Flush complete for sensor id ");
      Serial.println(byte1);
      break;
    case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
      //printf("%s; T: %u.%09u; Sample rate changed for sensor id %u\r\n", event_text, s,ns, byte1);
      Serial.print(event_text);
      Serial.print(" Sample rate changed for sensor id ");
      Serial.println(byte1);
      break;
    case BHY2_META_EVENT_POWER_MODE_CHANGED:
      //printf("%s; T: %u.%09u; Power mode changed for sensor id %u\r\n", event_text, s,ns, byte1);
      Serial.print(event_text);
      Serial.print(" Power mode changed for sensor id ");
      Serial.println(byte1);
      break;
    case BHY2_META_EVENT_ALGORITHM_EVENTS:
      //printf("%s; T: %u.%09u; Algorithm event\r\n", event_text, s, ns);
      Serial.print(event_text);
      Serial.println(" Algorithm event");
      break;
    case BHY2_META_EVENT_SENSOR_STATUS:
      //printf("%s; T: %u.%09u; Accuracy for sensor id %u changed to %u\r\n", event_text,s, ns, byte1, byte2);
      Serial.print(event_text);
      Serial.print(" Accuracy for sensor id ");
      Serial.print(byte1);
      Serial.print(" changed to ");
      Serial.println(byte2);
      break;
    case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
      //printf("%s; T: %u.%09u; BSX event (do steps main)\r\n", event_text, s, ns);
      Serial.print(event_text);
      Serial.println(" Algorithm event");
      break;
    case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
      //printf("%s; T: %u.%09u; BSX event (do steps calib)\r\n", event_text, s, ns);
      Serial.print(event_text);
      Serial.println(" BSX event (do steps calib)");
      break;
    case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
      //printf("%s; T: %u.%09u; BSX event (get output signal)\r\n", event_text, s, ns);
      Serial.print(event_text);
      Serial.println(" BSX event (get output signal)");
      break;
    case BHY2_META_EVENT_SENSOR_ERROR:
      //printf("%s; T: %u.%09u; Sensor id %u reported error 0x%02X\r\n", event_text, s, ns,byte1, byte2);
      Serial.print(event_text);
      Serial.print(" Sensor id ");
      Serial.print(byte1);
      Serial.print(" reported error 0x ");
      Serial.println(byte2, HEX);
      break;
    case BHY2_META_EVENT_FIFO_OVERFLOW:
      //printf("%s; T: %u.%09u; FIFO overflow\r\n", event_text, s, ns);
      Serial.print(event_text);
      Serial.println(" FIFO overflow");
      break;
    case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
      //printf("%s; T: %u.%09u; Dynamic range changed for sensor id %u\r\n", event_text, s,ns, byte1);
      Serial.print(event_text);
      Serial.print(" Dynamic range changed for sensor id ");
      Serial.println(byte1);
      break;
    case BHY2_META_EVENT_FIFO_WATERMARK:
      //printf("%s; T: %u.%09u; FIFO watermark reached\r\n", event_text, s, ns);
      Serial.print(event_text);
      Serial.println(" FIFO watermark reached");
      break;
    case BHY2_META_EVENT_INITIALIZED:
      //printf("%s; T: %u.%09u; Firmware initialized. Firmware version %u\r\n", event_text,s, ns,
            //((uint16_t )byte2 << 8) | byte1);
      Serial.print(event_text);
      Serial.print(" Firmware initialized. Firmware version ");
      Serial.println(((uint16_t )byte2 << 8) | byte1);
      break;
    case BHY2_META_TRANSFER_CAUSE:
      //printf("%s; T: %u.%09u; Transfer cause for sensor id %u\r\n", event_text, s, ns,byte1);
      Serial.print(event_text);
      Serial.print(" Transfer cause for sensor id ");
      Serial.println(byte1);
      break;
    case BHY2_META_EVENT_SENSOR_FRAMEWORK:
      //printf("%s; T: %u.%09u; Sensor framework event for sensor id %u\r\n", event_text,s, ns, byte1);
      Serial.print(event_text);
      Serial.print(" Sensor framework event for sensor id ");
      Serial.println(byte1);
      break;
    case BHY2_META_EVENT_RESET:
      //printf("%s; T: %u.%09u; Reset event\r\n", event_text, s, ns);
      Serial.print(event_text);
      Serial.println(" Reset event");
      break;
    case BHY2_META_EVENT_SPACER:
      break;
    default:
      //printf("%s; T: %u.%09u; Unknown meta event with id: %u\r\n", event_text, s, ns,meta_event_type);
      Serial.print(event_text);
      Serial.print(" Unknown meta event with id: ");
      Serial.println(meta_event_type);
      break;
  }
}
//----------------------------------------------------------------------------
void parseDebugMessage(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
  uint32_t s, ns;
  convertTime(*callback_info->time_stamp, &s, &ns);
    Serial.print("[DEBUG MSG]; flag: 0x");
    Serial.print(callback_info->data_ptr[0]);
    Serial.print(" data: ");
    Serial.println((char*)&callback_info->data_ptr[1]);
}
//----------------------------------------------------------------------------
void convertTime(uint64_t time_ticks, uint32_t *s, uint32_t *ns)
{
    uint64_t timestamp = time_ticks; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(timestamp / UINT64_C(1000000000));
    *ns = (uint32_t)(timestamp - ((*s) * UINT64_C(1000000000)));
}
//----------------------------------------------------------------------------
// Function to parse the data from the FIFO buffer and add it to the queue
// BISOGNA MODIFICARE IN MODO DA METTERE DATI IN dataBuffer!!!
void parseData(const struct bhy2_fifo_parse_data_info *fifoData, void *arg)
{
  int8_t sz;

  sz = fifoData->data_size - 1;
  if (sz <= SENSOR_DATA_FIXED_LENGTH) {
      SensorDataPacket sensorData;
      sensorData.sensorId = fifoData->sensor_id;
      sensorData.size = sz + 1;
      if (sz > 0)
          memcpy(&sensorData.data, fifoData->data_ptr, sz);

      addSensorData(sensorData);
  } else {
      SensorLongDataPacket sensorDataLong;
      sensorDataLong.sensorId = fifoData->sensor_id;
      sz = (sz <= SENSOR_LONG_DATA_FIXED_LENGTH) ? sz : SENSOR_LONG_DATA_FIXED_LENGTH;
      sensorDataLong.size = sz + 1;

      memcpy(&sensorDataLong.data, fifoData->data_ptr, sz);
      addLongSensorData(sensorDataLong);
  }
}
//----------------------------------------------------------------------------
void addSensorData(SensorDataPacket &sensorData)
{
  // Overwrites oldest data when fifo is full
  _sensorQueue.push(sensorData);
  // Alternative: handle the full queue by storing it in flash
  sensorManager.process(sensorData);
}
void addLongSensorData(SensorLongDataPacket &sensorDataLong)
{
  // Overwrites oldest data when fifo is full
  _longSensorQueue.push(sensorDataLong);
  // Alternative: handle the full queue by storing it in flash
  sensorManager.process(sensorDataLong);
}