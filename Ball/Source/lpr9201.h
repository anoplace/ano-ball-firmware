/**
 * @file parser.h
 * @author Yuuki Taguchi
 */

#ifndef LPR9201_PARSER_H_
#define LPR9201_PARSER_H_

#include <inttypes.h>
#include <string.h>
#include "result/result.h"
#include "result/result_const.h"

/**
 * データパース
 */

static const uint8_t START_BYTE1 = 0x5A;
static const uint8_t START_BYTE2 = 0xA5;
static const uint8_t START_BYTE_LENGTH = 2;

static const uint8_t COMMAND_BYTE_LENGTH = 1;
static const uint8_t CHECKSUM_BYTE_LENGTH = 1;

static const uint16_t RECEIVE_DATA_BUFFER_LENGTH = 512;
uint8_t receiveData[RECEIVE_DATA_BUFFER_LENGTH];
uint16_t receiveDataLength;
bool isStart;


/**
 *
 */
void lpr9201_parser_init() {
  this->receiveDataLength = 0;
  this->isStart = false;
}

/**
 * パケットを受信
 *
 * @param data
 * @param result
 * @return パースできたか
 */
bool_t lpr9201_parser_parse(uint8 data, result::Result* result) {
  bool_t isParsed = false;

  // header check
  if (this->receiveData[0] == START_BYTE1 && data == START_BYTE2 &&
      !this->isStart) {
    this->receiveDataLength = 1;
    this->isStart = true;
  }

  this->receiveData[this->receiveDataLength] = data;

  if (this->isStart) {
    this->receiveDataLength++;

    if (receiveDataLength > RECEIVE_DATA_BUFFER_LENGTH) {
      this->receiveDataLength = 0;
      this->isStart = false;
    }

    if (this->receiveDataLength > START_BYTE_LENGTH) {
      uint8_t command = this->receiveData[2];

      if (result::ResultConst::getDataLengthByteSize(command) > 0) {
        uint8_t dataLengthByteSize =
            result::ResultConst::getDataLengthByteSize(command);

        if (this->receiveDataLength >
            (uint16_t)(START_BYTE_LENGTH + dataLengthByteSize)) {
          uint16_t dataLength = 0;
          for (int i = 0; i < dataLengthByteSize; i++) {
            uint8_t byteData =
                this->receiveData[START_BYTE_LENGTH + COMMAND_BYTE_LENGTH + i];
            uint8_t shiftCount = 8 * (dataLengthByteSize - i - 1);
            dataLength |= byteData << shiftCount;
          }

          if (this->receiveDataLength >=
              START_BYTE_LENGTH + COMMAND_BYTE_LENGTH + dataLengthByteSize +
                  dataLength + CHECKSUM_BYTE_LENGTH) {
            // calculate checksum
            uint8_t checksum = 0;
            for (uint16_t i = 0; i < this->receiveDataLength - 1;
                 i++) {  // checksumは除く
              checksum ^= this->receiveData[i];
            }

            // check checksum
            if (checksum == this->receiveData[this->receiveDataLength - 1]) {
              uint8_t startIndex =
                  START_BYTE_LENGTH + COMMAND_BYTE_LENGTH + dataLengthByteSize;

              result->resultCode = command;
              result->datas = (uint8_t*)memmove(
                  this->receiveData, &this->receiveData[startIndex],
                  this->receiveDataLength - startIndex - CHECKSUM_BYTE_LENGTH);
              result->dataLength =
                  this->receiveDataLength - startIndex - CHECKSUM_BYTE_LENGTH;

              isParsed = true;
            }

            this->receiveDataLength = 0;
            this->isStart = false;
          }
        }
      } else {
        this->receiveDataLength = 0;
        this->isStart = false;
      }
    }
  }

  return isParsed;
}

#endif
