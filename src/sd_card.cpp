/* Code based on the SdInfo example from the SdFat library:
 * https://github.com/greiman/SdFat/blob/master/examples/SdInfo/SdInfo.ino
 *
 * https://gurumeditation.org/1342/sd-memory-card-register-decoder/
 * https://archive.goughlui.com/static/multicid.htm
 */

#include "sd_card.h"

// Set DISABLE_CS_PIN to disable a second SPI device. Set -1 to disable this feature.
//const int8_t DISABLE_CS_PIN = D6;
const int8_t DISABLE_CS_PIN = -1;

// SD card configuration

#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_HZ(spi_freq)) // Max speed 48 MHz

//------------------------------------------------------------------------------
SdFs sd;
SdFile myFile;
cid_t cid;
csd_t csd;
scr_t scr;
uint8_t cmd6Data[64];
uint32_t eraseSize;
uint32_t ocr;
static ArduinoOutStream cout(Serial);
//------------------------------------------------------------------------------
void hexDmp(void* reg, uint8_t size) {
  uint8_t* u8 = reinterpret_cast<uint8_t*>(reg);
  cout << hex << noshowbase;
  for (size_t i = 0; i < size; i++) {
    cout << setw(2) << setfill('0') << int(u8[i]);
  }
  cout << dec << endl;
}
//------------------------------------------------------------------------------
void cidDmp() {
  cout << F("\nManufacturer ID: ");
  cout << uppercase << showbase << hex << int(cid.mid) << dec << endl;
  cout << F("OEM ID: ") << cid.oid[0] << cid.oid[1] << endl;
  cout << F("Product: ");
  for (uint8_t i = 0; i < 5; i++) {
    cout << cid.pnm[i];
  }
  cout << F("\nRevision: ") << cid.prvN() << '.' << cid.prvM() << endl;
  cout << F("Serial number: ") << hex << cid.psn() << dec << endl;
  cout << F("Manufacturing date: ");
  cout << cid.mdtMonth() << '/' << cid.mdtYear() << endl;
  cout << F("CID HEX: ");
  hexDmp(&cid, sizeof(cid));
}
//------------------------------------------------------------------------------
void clearSerialInput() {
  uint32_t m = micros();
  do {
    if (Serial.read() >= 0) {
      m = micros();
    }
  } while (micros() - m < 10000);
}
//------------------------------------------------------------------------------
void csdDmp() {
  eraseSize = csd.eraseSize();
  cout << F("\ncardSize: ") << 0.000512 * csd.capacity();
  cout << F(" MB (MB = 1,000,000 bytes)\n");

  cout << F("flashEraseSize: ") << int(eraseSize) << F(" blocks\n");
  cout << F("eraseSingleBlock: ");
  if (csd.eraseSingleBlock()) {
    cout << F("true\n");
  } else {
    cout << F("false\n");
  }
  cout << F("dataAfterErase: ");
  if (scr.dataAfterErase()) {
    cout << F("ones\n");
  } else {
    cout << F("zeros\n");
  }
  cout << F("CSD HEX: ");
  hexDmp(&csd, sizeof(csd));
}
//------------------------------------------------------------------------------
void errorPrint() {
  if (sd.sdErrorCode()) {
    cout << F("SD errorCode: ") << hex << showbase;
    printSdErrorSymbol(&Serial, sd.sdErrorCode());
    cout << F(" = ") << int(sd.sdErrorCode()) << endl;
    cout << F("SD errorData = ") << int(sd.sdErrorData()) << dec << endl;
  }
}
//------------------------------------------------------------------------------
bool mbrDmp() {
  MbrSector_t mbr;
  bool valid = true;
  if (!sd.card()->readSector(0, (uint8_t*)&mbr)) {
    cout << F("\nread MBR failed.\n");
    errorPrint();
    return false;
  }
  cout << F("\nSD Partition Table\n");
  cout << F("part,boot,bgnCHS[3],type,endCHS[3],start,length\n");
  for (uint8_t ip = 1; ip < 5; ip++) {
    MbrPart_t* pt = &mbr.part[ip - 1];
    if ((pt->boot != 0 && pt->boot != 0X80) ||
        getLe32(pt->relativeSectors) > csd.capacity()) {
      valid = false;
    }
    cout << int(ip) << ',' << uppercase << showbase << hex;
    cout << int(pt->boot) << ',';
    for (int i = 0; i < 3; i++) {
      cout << int(pt->beginCHS[i]) << ',';
    }
    cout << int(pt->type) << ',';
    for (int i = 0; i < 3; i++) {
      cout << int(pt->endCHS[i]) << ',';
    }
    cout << dec << getLe32(pt->relativeSectors) << ',';
    cout << getLe32(pt->totalSectors) << endl;
  }
  if (!valid) {
    cout << F("\nMBR not valid, assuming Super Floppy format.\n");
  }
  return true;
}
//------------------------------------------------------------------------------
void dmpVol() {
  cout << F("\nScanning FAT, please wait.\n");
  int32_t freeClusterCount = sd.freeClusterCount();
  if (sd.fatType() <= 32) {
    cout << F("\nVolume is FAT") << int(sd.fatType()) << endl;
  } else {
    cout << F("\nVolume is exFAT\n");
  }
  cout << F("sectorsPerCluster: ") << sd.sectorsPerCluster() << endl;
  cout << F("fatStartSector:    ") << sd.fatStartSector() << endl;
  cout << F("dataStartSector:   ") << sd.dataStartSector() << endl;
  cout << F("clusterCount:      ") << sd.clusterCount() << endl;
  cout << F("freeClusterCount:  ");
  if (freeClusterCount >= 0) {
    cout << freeClusterCount << endl;
  } else {
    cout << F("failed\n");
    errorPrint();
  }
}
//------------------------------------------------------------------------------
void printCardType() {
  cout << F("\nCard type: ");

  switch (sd.card()->type()) {
    case SD_CARD_TYPE_SD1:
      cout << F("SD1\n");
      break;

    case SD_CARD_TYPE_SD2:
      cout << F("SD2\n");
      break;

    case SD_CARD_TYPE_SDHC:
      if (csd.capacity() < 70000000) {
        cout << F("SDHC\n");
      } else {
        cout << F("SDXC\n");
      }
      break;

    default:
      cout << F("Unknown\n");
  }
}
//------------------------------------------------------------------------------
void printConfig(SdSpiConfig config) {
  if (DISABLE_CS_PIN < 0) {
    cout << F(
        "\nAssuming the SD is the only SPI device.\n"
        "Edit DISABLE_CS_PIN to disable an SPI device.\n");
  } else {
    cout << F("\nDisabling SPI device on pin ");
    cout << int(DISABLE_CS_PIN) << endl;
    pinMode(DISABLE_CS_PIN, OUTPUT);
    digitalWrite(DISABLE_CS_PIN, HIGH);
  }
  cout << F("\nAssuming the SD chip select pin is: ") << int(config.csPin);
  cout << F("\nEdit SD_CS_PIN to change the SD chip select pin.\n");
}
//------------------------------------------------------------------------------
void printConfig(SdioConfig config) {
  (void)config;
  cout << F("Assuming an SDIO interface.\n");
}
//-----------------------------------------------------------------------------
void sdSetup() {
  digitalWrite(sensorCS, HIGH);
  cout << F("SdFat version: ") << SD_FAT_VERSION_STR << endl;
  printConfig(SD_CONFIG);
  //delay(2000); // Debug delay, lower/remove in final build
  uint32_t t = millis();
  if (!sd.cardBegin(SD_CONFIG)) {
    cout << F(
        "\nSD initialization failed.\n"
        "Do not reformat the card!\n"
        "Is the card correctly inserted?\n"
        "Is there a wiring/soldering problem?\n");
    if (isSpi(SD_CONFIG)) {
      cout << F(
          "Is SD_CS_PIN set to the correct value?\n"
          "Does another SPI device need to be disabled?\n");
    }
    errorPrint();
    return;
  }
  t = millis() - t;
  cout << F("init time: ") << dec << t << " ms" << endl;
  if (!sd.volumeBegin()) {
    cout << F("\nvolumeBegin failed. Is the card formatted?\n");
    errorPrint();
    return;
  }
  digitalWrite(sensorCS, LOW);
}
//------------------------------------------------------------------------------
void sdCheck() {
  // Read any existing Serial data.
  clearSerialInput();

  // F stores strings in flash to save RAM
  cout << F("\nType any character to proceed with the SD card check\n");
  while (!Serial.available()) {
    yield();
  }

  if (!sd.card()->readCID(&cid) || !sd.card()->readCSD(&csd) ||
      !sd.card()->readOCR(&ocr) || !sd.card()->readSCR(&scr)) {
    cout << F("readInfo failed\n");
    errorPrint();
    return;
  }
  printCardType();
  cout << F("sdSpecVer: ") << 0.01 * scr.sdSpecVer() << endl;
  cout << F("HighSpeedMode: ");
  if (scr.sdSpecVer() > 101 && sd.card()->cardCMD6(0X00FFFFFF, cmd6Data) &&
      (2 & cmd6Data[13])) {
    cout << F("true\n");
  } else {
    cout << F("false\n");
  }
  cidDmp();
  csdDmp();
  cout << F("\nOCR: ") << uppercase << showbase;
  cout << hex << ocr << dec << endl;
  if (!mbrDmp()) {
    return;
  }
  dmpVol();
  clearSerialInput();
  cout << F("\nType any character to continue\n");
  while (!Serial.available()) {
    yield();
  }
}
//------------------------------------------------------------------------------
void sdTestWrite() {
  // Test writing to SD card
  if (!myFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("\nopening test.txt for write failed");
  }
  // if the file opened okay, write to it:
  Serial.print("\nWriting to test.txt...");
  myFile.println("If you are reading this, writing and reading to the SD card works!");
  // close the file:
  myFile.close();
  Serial.println("done.");
  // re-open the file for reading:
  if (!myFile.open("test.txt", O_RDWR)) {
    sd.errorHalt("opening test.txt for read failed");
  }
  Serial.println("test.txt:");

  // read from the file until there's nothing else in it:
  int data;
  while ((data = myFile.read()) >= 0) {
    Serial.write(data);
  }
  // remove the file:
  if (!myFile.remove()) sd.errorHalt("Error file.remove");
}
//------------------------------------------------------------------------------
void sdWrite(String line, const char* filename, bool keepOpen) {
  if (!myFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("\nopening for write failed");
  }
  // if the file opened okay, write to it:
  myFile.println(line);
  // close the file:
  if (!keepOpen)
  {
    myFile.close();
  }  
}