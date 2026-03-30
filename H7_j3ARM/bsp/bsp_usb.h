

#ifndef __BSP_USB_H__
#define __BSP_USB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

/* 上位机 -> 下位机：发送目标三维坐标 */
#define USB_CMD_ARM_SET_XYZ        0x10U

/* 下位机 -> 上位机：返回解算结果状态 */
#define USB_CMD_ARM_IK_RESULT      0x90U

/* 帧格式常量 */
#define USB_FRAME_HEAD1            0xA5U
#define USB_FRAME_HEAD2            0x5AU
#define USB_FRAME_TAIL             0xFFU
#define USB_FRAME_OVERHEAD         7U      /* 2头 + 1长度 + 1命令 + 2CRC + 1尾 */
#define USB_FRAME_MAX_DATA_LEN     255U
#define USB_FRAME_BUF_SIZE         300U

extern uint8_t usb_Buf[USB_FRAME_BUF_SIZE];


/* 发送接口 */
void     SendByte(uint8_t data);
uint8_t  Send(const uint8_t *data, uint16_t len);
uint16_t CRC16_Check(const uint8_t *data, uint16_t len);
uint8_t  Send_Cmd_Data(uint8_t cmd, const uint8_t *datas, uint8_t len);

/* 调试统计 */
uint32_t USB_GetSendDropFrames(void);

/* 接收解析 */
void Data_Analysis(uint8_t cmd, const uint8_t* datas, uint8_t len);
void Receive(uint8_t bytedata);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_USB_H__ */
