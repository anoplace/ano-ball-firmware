#ifndef PCA9685_H_INCLUDED
#define PCA9685_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

// slave address
#define PCA9685_BASE_ADDRESS 0x40
#define PCA9685_ALLCALL_ADDRESS 0x70
#define PCA9685_SOFTWARE_RESET_ADDRESS 0x06

// register map
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_SUBADR1 0x02
#define PCA9685_SUBADR2 0x03
#define PCA9685_SUBADR3 0x04
#define PCA9685_ALLCALLADR 0x05

#define PCA9685_LED0_ON_L 0x06
#define PCA9685_LED0_ON_H 0x07
#define PCA9685_LED0_OFF_L 0x08
#define PCA9685_LED0_OFF_H 0x09

#define PCA9685_LED1_ON_L 0x0A
#define PCA9685_LED1_ON_H 0x0B
#define PCA9685_LED1_OFF_L 0x0C
#define PCA9685_LED1_OFF_H 0x0D

#define PCA9685_LED2_ON_L 0x0E
#define PCA9685_LED2_ON_H 0x0F
#define PCA9685_LED2_OFF_L 0x10
#define PCA9685_LED2_OFF_H 0x11

#define PCA9685_LED3_ON_L 0x12
#define PCA9685_LED3_ON_H 0x13
#define PCA9685_LED3_OFF_L 0x14
#define PCA9685_LED3_OFF_H 0x15

#define PCA9685_LED4_ON_L 0x16
#define PCA9685_LED4_ON_H 0x17
#define PCA9685_LED4_OFF_L 0x18
#define PCA9685_LED4_OFF_H 0x19

#define PCA9685_LED5_ON_L 0x1A
#define PCA9685_LED5_ON_H 0x1B
#define PCA9685_LED5_OFF_L 0x1C
#define PCA9685_LED5_OFF_H 0x1D

#define PCA9685_LED6_ON_L 0x1E
#define PCA9685_LED6_ON_H 0x1F
#define PCA9685_LED6_OFF_L 0x20
#define PCA9685_LED6_OFF_H 0x21

#define PCA9685_LED7_ON_L 0x22
#define PCA9685_LED7_ON_H 0x23
#define PCA9685_LED7_OFF_L 0x24
#define PCA9685_LED7_OFF_H 0x25

#define PCA9685_LED8_ON_L 0x26
#define PCA9685_LED8_ON_H 0x27
#define PCA9685_LED8_OFF_L 0x28
#define PCA9685_LED8_OFF_H 0x29

#define PCA9685_LED9_ON_L 0x2A
#define PCA9685_LED9_ON_H 0x2B
#define PCA9685_LED9_OFF_L 0x2C
#define PCA9685_LED9_OFF_H 0x2D

#define PCA9685_LED10_ON_L 0x2E
#define PCA9685_LED10_ON_H 0x2F
#define PCA9685_LED10_OFF_L 0x30
#define PCA9685_LED10_OFF_H 0x31

#define PCA9685_LED11_ON_L 0x32
#define PCA9685_LED11_ON_H 0x33
#define PCA9685_LED11_OFF_L 0x34
#define PCA9685_LED11_OFF_H 0x35

#define PCA9685_LED12_ON_L 0x36
#define PCA9685_LED12_ON_H 0x37
#define PCA9685_LED12_OFF_L 0x38
#define PCA9685_LED12_OFF_H 0x39

#define PCA9685_LED13_ON_L 0x3A
#define PCA9685_LED13_ON_H 0x3B
#define PCA9685_LED13_OFF_L 0x3C
#define PCA9685_LED13_OFF_H 0x3D

#define PCA9685_LED14_ON_L 0x3E
#define PCA9685_LED14_ON_H 0x3F
#define PCA9685_LED14_OFF_L 0x40
#define PCA9685_LED14_OFF_H 0x41

#define PCA9685_LED15_ON_L 0x42
#define PCA9685_LED15_ON_H 0x43
#define PCA9685_LED15_OFF_L 0x44
#define PCA9685_LED15_OFF_H 0x45

#define PCA9685_ALL_LED_ON_L 0xFA
#define PCA9685_ALL_LED_ON_H 0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD

#define PCA9685_PRESCALE 0xFE
#define PCA9685_TEST_MODE 0xFF

// mode1 bits
#define PCA9685_MODE1_RESTART 7
#define PCA9685_MODE1_EXTCLK 6
#define PCA9685_MODE1_AI 5
#define PCA9685_MODE1_SLEEP 4
#define PCA9685_MODE1_SUB1 3
#define PCA9685_MODE1_SUB2 2
#define PCA9685_MODE1_SUB3 1
#define PCA9685_MODE1_ALLCALL 0

// mode2 bits
#define PCA9685_MODE2_INVERT 4
#define PCA9685_MODE2_OCH 3
#define PCA9685_MODE2_OUTDRV 2
#define PCA9685_MODE2_OUTNE1 1
#define PCA9685_MODE2_OUTNE0 0

// prescale
#define PCA9685_PRESCALE_INTERNAL_OSC 25000000  // 25MHz
#define PCA9685_PRESCALE_DEFAULT_FREQUENCY 200  // 200Hz

// led number
#define PCA9685_LED_NUMBER 16
#define PCA9685_LED_RGBW_NUMBER 4

typedef struct {
  uint8 slaveAddress;  //
} tsPCA9685;

PUBLIC void vPCA9685_Init(tsPCA9685 *psPCA9685);
PUBLIC void vPCA9685_setLedRaw(tsPCA9685 *psPCA9685, uint8 index, uint16 value);
PUBLIC void vPCA9685_setLed(tsPCA9685 *psPCA9685, uint8 index, float fPercent);
PUBLIC void vPCA9685_setAllLedRaw(tsPCA9685 *psPCA9685, uint16 value);
PUBLIC void vPCA9685_setAllLed(tsPCA9685 *psPCA9685, float fPercent);
PUBLIC void vPCA9685_setRgbwLedRaw(tsPCA9685 *psPCA9685, uint8 u8Index,
                                   uint16 u16Red, uint16 u16Green,
                                   uint16 u16Blue, uint16 u16White);
PUBLIC void vPCA9685_setRgbwLed(tsPCA9685 *psPCA9685, uint8 u8Index,
                                uint32 u32Color);
PUBLIC void vPCA9685_prescale(tsPCA9685 *psPCA9685, uint32 u32Clock,
                              uint16 u16Frequency);
PUBLIC void vPCA9685_testMode(tsPCA9685 *psPCA9685);
PUBLIC void vPCA9685_reset(tsPCA9685 *psPCA9685);
PUBLIC void vPCA9685_readRegister(tsPCA9685 *psPCA9685, uint8 u8Register,
                                  uint8 *u8Data);
PUBLIC void vPCA9685_writeRegister(tsPCA9685 *psPCA9685, uint8 u8Register,
                                   uint8 u8Value);
PUBLIC void vPCA9685_readRegisters(tsPCA9685 *psPCA9685, uint8 u8Register,
                                   uint8 *pu8Data, uint8 u8Length);
PUBLIC void vPCA9685_writeRegisters(tsPCA9685 *psPCA9685, uint8 u8Register,
                                    uint8 *pu8Data, uint8 u8Length);

#if defined __cplusplus
}
#endif

#endif /* PCA9685_H_INCLUDED */
