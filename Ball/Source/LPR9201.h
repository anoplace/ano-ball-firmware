/**
 * @file parser.h
 * @author Yuuki Taguchi
 */

#ifndef LPR9201_PARSER_H_
#define LPR9201_PARSER_H_

#include <inttypes.h>
#include <string.h>
//#include "result/result.h"
//#include "result/result_const.h"

/**
 * データパース
 */

const uint8 START_BYTE1 = 0x5A;
const uint8 START_BYTE2 = 0xA5;
const uint8 START_BYTE_LENGTH = 2;

const uint8 COMMAND_BYTE_LENGTH = 1;
const uint8 CHECKSUM_BYTE_LENGTH = 1;

const uint16 RECEIVE_DATA_BUFFER_LENGTH = 512;
// uint8 receiveData[512];
// uint16 receiveDataLength;
// bool_t isStart;

typedef struct {
  uint8 resultCode;
  uint8 receiveData[512];
  uint16_t receiveDataLength;
  bool_t isStart;

  uint16_t dataOffset;
  uint16_t dataLength;
} Result;

/**
 *
 */
void lpr9201_parser_init(Result *result) {
  result->receiveDataLength = 0;
  result->isStart = FALSE;
}

/**
 * データ長バイトサイズ一覧
 */
static uint8 getDataLengthByteSize(uint8 command) {
  return command == 0x83 ? 2 : 1;
}

/**
 * パケットを受信
 *
 * @param data
 * @param result
 * @return パースできたか
 */
bool_t lpr9201_parser_parse(uint8 data, Result *result) {
  bool_t isParsed = FALSE;

  // header check
  if (result->receiveData[0] == START_BYTE1 && data == START_BYTE2 &&
      !result->isStart) {
    result->receiveDataLength = 1;
    result->isStart = TRUE;
  }

  result->receiveData[result->receiveDataLength] = data;

  if (result->isStart) {
    result->receiveDataLength++;

    if (result->receiveDataLength > RECEIVE_DATA_BUFFER_LENGTH) {
      result->receiveDataLength = 0;
      result->isStart = FALSE;
    }

    if (result->receiveDataLength > START_BYTE_LENGTH) {
      uint8 command = result->receiveData[2];

      uint8 dataLengthByteSize = getDataLengthByteSize(command);

      if (result->receiveDataLength >
          (uint16)(START_BYTE_LENGTH + dataLengthByteSize)) {
        uint16 dataLength = 0;
        for (int i = 0; i < dataLengthByteSize; i++) {
          uint8 byteData =
              result->receiveData[START_BYTE_LENGTH + COMMAND_BYTE_LENGTH + i];
          uint8 shiftCount = 8 * (dataLengthByteSize - i - 1);
          dataLength |= byteData << shiftCount;
        }

        if (result->receiveDataLength >=
            START_BYTE_LENGTH + COMMAND_BYTE_LENGTH + dataLengthByteSize +
                dataLength + CHECKSUM_BYTE_LENGTH) {
          // calculate checksum
          uint8 checksum = 0;
          for (uint16 i = 0; i < result->receiveDataLength - 1;
               i++) {  // checksumは除く
            checksum ^= result->receiveData[i];
          }

          // check checksum
          if (checksum == result->receiveData[result->receiveDataLength - 1]) {
            uint8 startIndex =
                START_BYTE_LENGTH + COMMAND_BYTE_LENGTH + dataLengthByteSize;

            result->resultCode = command;
            /*
            result->datas = (uint8_t*)memmove(
                result->receiveData, &result->receiveData[startIndex],
                result->receiveDataLength - startIndex -
            CHECKSUM_BYTE_LENGTH);
            */
            result->dataOffset = startIndex;
            result->dataLength =
                result->receiveDataLength - startIndex - CHECKSUM_BYTE_LENGTH;

            isParsed = TRUE;
          }

          result->receiveDataLength = 0;
          result->isStart = FALSE;
        }
      }
    }
  }

  return isParsed;
}

#endif
