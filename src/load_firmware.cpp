// reference used: https://github.com/boschsensortec/BHI2xy_SensorAPI/blob/master/examples/load_firmware/load_firmware.c

#include "load_firmware.h"
//#include "firmwares/fw.h"
//#include "firmwares/bhi260_aux_bmm150_bmp390_bme688_flash.h"
#include "firmwares/BHI260AP_aux_BMM150_BMP390_BME688-flash.fw.h"
/* FIRMWARE UPLOAD:
    INFO:
    * only include one firmware at a time
    * modify the defines below depending on the firmware you want to use

    LIST OF FIRMWARES:
    ** bhi260_aux_bmm150_bmp390_bme688_flash.h       original firmware of Smart Sens 2 Click flash version (DEFAULT)
    * fw.h                                          arduino version of BHI260AP Nicla Sense ME firmware flash (do not use)
    * BHI260AP_aux_BMM150_BMP390_BME688-flash.fw.h  bosch original firmware of BHI260AP flash version (to be tested)
    * others...
*/

//#define FIRMWARE_ARRAY BHI260AP_NiclaSenseME_flash_fw
//#define FIRMWARE_SIZE  BHI260AP_NiclaSenseME_flash_fw_len
#define FIRMWARE_ARRAY bhy2_firmware_image
#define FIRMWARE_SIZE sizeof(bhy2_firmware_image)

static void print_api_error(int8_t rslt, struct bhy2_dev *dev)
{
    if (rslt != BHY2_OK)
    {
        printf("API error %d\r\n", rslt);
        /* printf("%s\r\n", get_api_error(rslt));
        if ((rslt == BHY2_E_IO) && (dev != NULL))
        {
            printf("%s\r\n", get_coines_error(dev->hif.intf_rslt));
            dev->hif.intf_rslt = BHY2_INTF_RET_SUCCESS;
        } */
        exit(0);
    }
}

static int8_t upload_firmware(struct bhy2_dev *dev)
{
    Serial.println("   Uploading firmware...");
    uint32_t incr = 256; // Max command packet size
    uint32_t len = FIRMWARE_SIZE;
    int8_t rslt = BHY2_OK;

    if ((incr % 4) != 0)
        incr = ((incr >> 2) + 1) << 2;

    for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr)
    {
        uint32_t chunk = incr;
        if (chunk > (len - i))
        {
            chunk = len - i;
            if ((chunk % 4) != 0)
                chunk = ((chunk >> 2) + 1) << 2;
        }
#ifdef UPLOAD_FIRMWARE_TO_FLASH
        rslt = bhy2_upload_firmware_to_flash_partly(&FIRMWARE_ARRAY[i], i, chunk, dev);
#else
        rslt = bhy2_upload_firmware_to_ram_partly(&FIRMWARE_ARRAY[i], len, i, chunk, dev);
#endif
        printf("%.2f%% complete\r", (float)(i + chunk) / (float)len * 100.0f);
    }
    printf("\n");
    Serial.println("   Firmware upload complete.");
    return rslt;
}

int bhi260ap_load_firmware(void)
{
    uint8_t product_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    struct bhy2_dev bhy2;
    uint8_t hintr_ctrl, hif_ctrl, boot_status;
    enum bhy2_intf intf;

#ifdef BHY2_USE_I2C
    intf = BHY2_I2C_INTERFACE;
#else
    intf = BHY2_SPI_INTERFACE;
#endif

    Serial.print("   Setting up interfaces...");
    setup_interfaces(true, intf);
    Serial.println(" Done");

#ifdef BHY2_USE_I2C
    rslt = bhy2_init(BHY2_I2C_INTERFACE, bhy2_i2c_read, bhy2_i2c_write, bhy2_delay_us, BHY2_RD_WR_LEN, NULL, &bhy2);
#else
    rslt = bhy2_init(BHY2_SPI_INTERFACE, bhy2_spi_read, bhy2_spi_write, bhy2_delay_us, BHY2_RD_WR_LEN, NULL, &bhy2);
#endif
    //print_api_error(rslt, &bhy2);

    rslt = bhy2_soft_reset(&bhy2);
    //print_api_error(rslt, &bhy2);

    rslt = bhy2_get_product_id(&product_id, &bhy2);
    //print_api_error(rslt, &bhy2);

    if (product_id != BHY2_PRODUCT_ID)
    {
        printf("Product ID read %X. Expected %X\r\n", product_id, BHY2_PRODUCT_ID);
        return -1;
    }
    printf("BHI260/BHA260 found. Product ID read %X\r\n", product_id);

    hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
    rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, &bhy2);
    //print_api_error(rslt, &bhy2);

    hif_ctrl = 0;
    rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &bhy2);
    //print_api_error(rslt, &bhy2);

    rslt = bhy2_get_boot_status(&boot_status, &bhy2);
    //print_api_error(rslt, &bhy2);

    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        uint8_t sensor_error;
        int8_t temp_rslt;
        printf("Loading firmware.\r\n");

#ifdef UPLOAD_FIRMWARE_TO_FLASH
        if (boot_status & BHY2_BST_FLASH_DETECTED)
        {
            uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
            uint32_t end_addr = start_addr + FIRMWARE_SIZE;
            printf("Flash detected. Erasing flash to upload firmware\r\n");
            rslt = bhy2_erase_flash(start_addr, end_addr, &bhy2);
            print_api_error(rslt, &bhy2);
        }
        else
        {
            printf("Flash not detected\r\n");
            return -2;
        }
#endif

        rslt = upload_firmware(&bhy2);
        temp_rslt = bhy2_get_error_value(&sensor_error, &bhy2);
        if (sensor_error)
            printf("%s\r\n", get_sensor_error_text(sensor_error));
        //print_api_error(rslt, &bhy2);
        //print_api_error(temp_rslt, &bhy2);

        Serial.println("   Booting from flash or RAM...");
#ifdef UPLOAD_FIRMWARE_TO_FLASH
        printf("Booting from Flash.\r\n");
        rslt = bhy2_boot_from_flash(&bhy2);
#else
        printf("Booting from RAM.\r\n");
        rslt = bhy2_boot_from_ram(&bhy2);
#endif
        Serial.println("   Boot Complete");
/* 
        temp_rslt = bhy2_get_error_value(&sensor_error, &bhy2);
        if (sensor_error)
            printf("%s\r\n", get_sensor_error_text(sensor_error));
        print_api_error(rslt, &bhy2);
        print_api_error(temp_rslt, &bhy2);

        rslt = bhy2_get_kernel_version(&version, &bhy2);
        print_api_error(rslt, &bhy2);
        if ((rslt == BHY2_OK) && (version != 0))
            printf("Boot successful. Kernel version %u.\r\n", version); */
    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");
        close_interfaces();
        return -3;
    }

    close_interfaces();
    return 0;
}