/* Reference used in this code:
 * https://github.com/arduino-libraries/Arduino_BHY2/tree/main
 */

#include "sensors.h"
#include "SensorTypes.h"

//------------------------------------------------------------------------------
//SPI sensors_spi(SPI_MOSI, SPI_MISO, SPI_SCK /*, SPI_PSELSS0 */);
enum bhy2_intf intf;
//static float dataBuffer[3][6]; // 3 axis, 6 sensors
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
	bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, BoschParser::parseMetaEvent, NULL, &_bhy2);
  bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, BoschParser::parseMetaEvent, NULL, &_bhy2);
  bhy2_register_fifo_parse_callback(BHY2_SYS_ID_DEBUG_MSG, BoschParser::parseDebugMessage, NULL, &_bhy2);
  Serial.println("System callbacks registered.");

  /* ret = bhy2_get_and_process_fifo(_workBuffer, WORK_BUFFER_SIZE, &_bhy2); // EXECUTION STOPS HERE!!!
	Serial.println(get_api_error(ret)); */

	// Register sensor callbacks
  // All sensors' data are handled in the same generic way
  Serial.print("Registering sensor callbacks... ");
  for (uint8_t i = 1; i < BHY2_SENSOR_ID_MAX; i++) {
    bhy2_register_fifo_parse_callback(i, parseData, NULL, &_bhy2);
  }

  bhy2_update_virtual_sensor_list(&_bhy2);
  bhy2_get_virt_sensor_list(&_bhy2);
  Serial.println("Sensor callbacks registered.");
  
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
// Function to read the FIFOs and parse the data in the BMI260AP
int8_t sensorUpdate() {
  // Probabily add update from Nicla!!! PROBLEM HERE!!!
  sensortec.update();

  /* int8_t result = BHY2_OK;
  result = bhy2_get_and_process_fifo(NULL, 0, &_bhy2);
        if (result != BHY2_OK)
        {
            printf("Errore FIFO: %s\n", get_api_error(result));
        }
  return result; */
}
/* void dataBufferPrint() {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++) {
      Serial.print("i: " + String(i) + " - j: " + String(j) + " - ");
      Serial.print(dataBuffer[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
} */
void spiTest() {
    // Set SD chip select pin high
    digitalWrite(SD_CS_PIN, HIGH);
    // Set sensor chip select pin low
    digitalWrite(sensorCS, LOW);
    
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
    
    // Set sensor chip select pin high
    digitalWrite(sensorCS, HIGH);
    // Set SD chip select pin low
    digitalWrite(SD_CS_PIN, LOW);
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