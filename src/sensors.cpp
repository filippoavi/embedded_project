#include "sensors.h"

#define ACCELEROMETER_ID 0
#define GYROSCOPE_ID 1
#define MAGNETOMETER_ID 2
#define TEMPERATURE_ID 3
#define HUMIDITY_ID 4
#define PRESSURE_ID 5
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define wait_us(us) _wait_us_inline(us)

arduino::MbedSPI spi(SPI_MOSI, SPI_MISO, SPI_SCK /*, SPI_PSELSS0 */);

//------------------------------------------------------------------------------
/* SensorXYZ accel(SENSOR_ID_ACC);
SensorXYZ gyro(SENSOR_ID_GYRO);
SensorXYZ mag(SENSOR_ID_MAG);
SensorQuaternion rotation(SENSOR_ID_RV);
Sensor temperature(SENSOR_ID_TEMP);
Sensor gas(SENSOR_ID_GAS);
Sensor pressure(SENSOR_ID_BARO);
SensorBSEC bsec(SENSOR_ID_BSEC); */
const int8_t sensorCS = D6;
float dataBuffer[3][6]; // 3 axis, 6 sensors
//------------------------------------------------------------------------------
// SPI communication functions
void setup_interfaces(bool reset_power, enum bhy2_intf intf)
{
    spi.begin();
}
int8_t bhy2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    digitalWrite(sensorCS, LOW);
    spi.transfer(reg_addr);
    spi.transfer(NULL, (uint16_t)length);
    spi.transfer((char*)reg_data, (uint16_t)length);
    digitalWrite(sensorCS, HIGH);

    return BHY2_INTF_RET_SUCCESS;
}

int8_t bhy2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;
    digitalWrite(sensorCS, LOW);
    spi.transfer(reg_addr);
    spi.transfer((char*)reg_data, (uint16_t)length);
    spi.transfer(NULL, 0);
    digitalWrite(sensorCS, HIGH);

    return BHY2_INTF_RET_SUCCESS;
}
void bhy2_delay_us(uint32_t us, void *private_data)
{
    (void)private_data;
    wait_us(us);
}
const char* get_api_error(int8_t error_code)
{
    const char *ret = " ";

    switch (error_code)
    {
        case BHY2_OK:
            break;
        case BHY2_E_NULL_PTR:
            ret = "[API Error] Null pointer";
            break;
        case BHY2_E_INVALID_PARAM:
            ret = "[API Error] Invalid parameter";
            break;
        case BHY2_E_IO:
            ret = "[API Error] IO error";
            break;
        case BHY2_E_MAGIC:
            ret = "[API Error] Invalid firmware";
            break;
        case BHY2_E_TIMEOUT:
            ret = "[API Error] Timed out";
            break;
        case BHY2_E_BUFFER:
            ret = "[API Error] Invalid buffer";
            break;
        case BHY2_E_INVALID_FIFO_TYPE:
            ret = "[API Error] Invalid FIFO type";
            break;
        case BHY2_E_INVALID_EVENT_SIZE:
            ret = "[API Error] Invalid Event size";
            break;
        case BHY2_E_PARAM_NOT_SET:
            ret = "[API Error] Parameter not set";
            break;
        default:
            ret = "[API Error] Unknown API error code";
    }

    return ret;
}
//------------------------------------------------------------------------------
void sensorSetup(struct bhy2_dev &bhy2_device) {
  //digitalWrite(sensorCS, LOW);
  /* BHY2.begin();
  accel.begin();        // Accelerometer
  gyro.begin();         // Gyroscope
  mag.begin();          // Magnetometer
  temperature.begin();  // Temperature
  pressure.begin();     // Pressure
  bsec.begin();         // BME sensor readings */
  int8_t result;
  // Configurazione dell'interfaccia hardware SPI
  setup_interfaces(true, BHY2_SPI_INTERFACE);
  // Inizializzazione del dispositivo BHY2
  result = bhy2_init(BHY2_SPI_INTERFACE, bhy2_spi_read, bhy2_spi_write, bhy2_delay_us, BHY2_RD_WR_LEN, NULL, &bhy2_device);
  if (result != BHY2_OK)
  {
      printf("Errore durante l'inizializzazione: %s\n", get_api_error(result));
  }
  // Registrazione delle callback per i sensori
  bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC, sensor_data_callback, NULL, &bhy2_device);
  bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GYRO, sensor_data_callback, NULL, &bhy2_device);
  bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_MAG, sensor_data_callback, NULL, &bhy2_device);
  bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_TEMP, sensor_data_callback, NULL, &bhy2_device);
  bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_HUM, sensor_data_callback, NULL, &bhy2_device);
  bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_BARO, sensor_data_callback, NULL, &bhy2_device);
  printf("Sensori inizializzati. Lettura dei dati...\n");
}
//------------------------------------------------------------------------------
void sensorReadSerial() {
  // Standalone Accelerometer values
/*   Serial.print("acc_X:");
  Serial.print(accel.x());
  Serial.print(",");
  Serial.print("acc_Y:");
  Serial.print(accel.y());
  Serial.print(",");
  Serial.print("acc_Z:");
  Serial.print(accel.z());
  Serial.print(",");
  // Standalone Gyroscope values
  Serial.print("gyro_X:");
  Serial.print(gyro.x());
  Serial.print(",");
  Serial.print("gyro_Y:");
  Serial.print(gyro.y());
  Serial.print(",");
  Serial.print("gyro_Z:");
  Serial.print(gyro.z());
  Serial.print(",");
  // Standalone Magnetometer values
  Serial.print("mag_X:");
  Serial.print(mag.x());
  Serial.print(",");
  Serial.print("mag_Y:");
  Serial.print(mag.y());
  Serial.print(",");
  Serial.print("mag_Z:");
  Serial.print(mag.z());
  Serial.print(",");
  // Standalone Temeprature values
  Serial.print(String("Temperature:") + String(temperature.value()) + String(","));
  // Standalone Pressure values
  Serial.print(String("Pressure:") + String(pressure.value()) + String(","));
  // Standalone BME sensor redings
  Serial.print(String("VOC:") + String(bsec.b_voc_eq()) + String(","));
  Serial.print(String("CO2:") + String(bsec.co2_eq()) + String(","));
  Serial.println(String("Humidity:") + String(bsec.comp_h())); */
}
//------------------------------------------------------------------------------
// Define a macro function to generate functions to read sensor values to String
#define SENSOR_READ_FUNC(name, sensor, axis) \
String name() { \
  return String(dataBuffer[axis][sensor]); \
}
// Functions to read sensor values
SENSOR_READ_FUNC(sensorReadAccX, ACCELEROMETER_ID, X_AXIS);
SENSOR_READ_FUNC(sensorReadAccY, ACCELEROMETER_ID, Y_AXIS);
SENSOR_READ_FUNC(sensorReadAccZ, ACCELEROMETER_ID, Z_AXIS);
SENSOR_READ_FUNC(sensorReadGyroX, GYROSCOPE_ID, X_AXIS);
SENSOR_READ_FUNC(sensorReadGyroY, GYROSCOPE_ID, Y_AXIS);
SENSOR_READ_FUNC(sensorReadGyroZ, GYROSCOPE_ID, Z_AXIS);
SENSOR_READ_FUNC(sensorReadMagX, MAGNETOMETER_ID, X_AXIS);
SENSOR_READ_FUNC(sensorReadMagY, MAGNETOMETER_ID, Y_AXIS);
SENSOR_READ_FUNC(sensorReadMagZ, MAGNETOMETER_ID, Z_AXIS);
SENSOR_READ_FUNC(sensorReadTemperature, TEMPERATURE_ID, 0);
SENSOR_READ_FUNC(sensorReadPressure, PRESSURE_ID, 0);
//SENSOR_READ_FUNC(sensorReadVOC, bsec, b_voc_eq);
//SENSOR_READ_FUNC(sensorReadCO2, bsec, co2_eq);
SENSOR_READ_FUNC(sensorReadHumidity, HUMIDITY_ID, 0);
//----------------------------------------------------------------------------
// Define a macro function to generate functions to get sensor values
#define SENSOR_GET_FUNC(name, sensor, axis) \
float name() { \
  return dataBuffer[axis][sensor]; \
}
// Functions to read sensor values
SENSOR_GET_FUNC(sensorGetAccX, ACCELEROMETER_ID, X_AXIS);
SENSOR_GET_FUNC(sensorGetAccY, ACCELEROMETER_ID, Y_AXIS);
SENSOR_GET_FUNC(sensorGetAccZ, ACCELEROMETER_ID, Z_AXIS);
SENSOR_GET_FUNC(sensorGetGyroX, GYROSCOPE_ID, X_AXIS);
SENSOR_GET_FUNC(sensorGetGyroY, GYROSCOPE_ID, Y_AXIS);
SENSOR_GET_FUNC(sensorGetGyroZ, GYROSCOPE_ID, Z_AXIS);
SENSOR_GET_FUNC(sensorGetMagX, MAGNETOMETER_ID, X_AXIS);
SENSOR_GET_FUNC(sensorGetMagY, MAGNETOMETER_ID, Y_AXIS);
SENSOR_GET_FUNC(sensorGetMagZ, MAGNETOMETER_ID, Z_AXIS);
SENSOR_GET_FUNC(sensorGetTemperature, TEMPERATURE_ID, 0);
SENSOR_GET_FUNC(sensorGetPressure, PRESSURE_ID, 0);
//SENSOR_GET_FUNC(sensorGetVOC, bsec, b_voc_eq);
//SENSOR_GET_FUNC(sensorGetCO2, bsec, co2_eq);
SENSOR_GET_FUNC(sensorGetHumidity, HUMIDITY_ID, 0);
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
int8_t sensorUpdate(struct bhy2_dev &bhy2_device) {
  int8_t result = BHY2_OK;
  result = bhy2_get_and_process_fifo(NULL, 0, &bhy2_device);
        if (result != BHY2_OK)
        {
            printf("Errore FIFO: %s\n", get_api_error(result));
        }
  return result;
}