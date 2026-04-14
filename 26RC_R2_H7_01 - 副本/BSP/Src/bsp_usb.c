#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include "arm_user.h"

void ArmIK_ComponentStep(float x, float y, float z);
static float USB_BytesToFloatLE(const uint8_t *buf);

/* 协议接收缓存 */
uint8_t usb_Buf[USB_FRAME_BUF_SIZE];

/* 发送失败统计：TX ring 空间不足等情况 */
static volatile uint32_t s_usbSendDropFrames = 0U;

/* 解析状态机上下文 */
static uint8_t  s_usbRxStep    = 0U;
static uint16_t s_usbRxCnt     = 0U;
static uint8_t  s_usbRxLen     = 0U;
static uint8_t  s_usbRxCmd     = 0U;
static uint8_t *s_usbRxDataPtr = 0;
static uint16_t s_usbRxCrc16   = 0U;


/*========================= 内部辅助函数 =========================*/

/* 解析器完全复位 */
static void USB_ParserReset(void)
{
    s_usbRxStep    = 0U;
    s_usbRxCnt     = 0U;
    s_usbRxLen     = 0U;
    s_usbRxCmd     = 0U;
    s_usbRxDataPtr = 0;
    s_usbRxCrc16   = 0U;
}

/* 把当前字节当作新的帧头1重新开始 */
static void USB_ParserRestartFromHead1(void)
{
    USB_ParserReset();
    s_usbRxStep = 1U;
    usb_Buf[s_usbRxCnt++] = USB_FRAME_HEAD1;
}

/* 往协议缓存里压一个字节，失败则复位 */
static uint8_t USB_ParserPushByte(uint8_t byte)
{
    if (s_usbRxCnt >= USB_FRAME_BUF_SIZE)
    {
        USB_ParserReset();
        return 0U;
    }

    usb_Buf[s_usbRxCnt++] = byte;
    return 1U;
}

/* 小端字节序转 float */
static float USB_BytesToFloatLE(const uint8_t *buf)
{
    union
    {
        uint8_t b[4];
        float   f;
    } u;

    u.b[0] = buf[0];
    u.b[1] = buf[1];
    u.b[2] = buf[2];
    u.b[3] = buf[3];

    return u.f;
}


/*========================= 发送部分 =========================*/

void SendByte(uint8_t data)
{
    (void)data;
    /* 这里保留空实现。
       你当前 USB CDC 发送已经走 Send()/Send_Cmd_Data() 了，
       不再需要逐字节调用这个函数。 */
}

uint8_t Send(const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len == 0U))
    {
        return 1U;
    }

    /* TX ring 空间不足，记录一次发送丢弃 */
    if (CDC_App_TxFree() < len)
    {
        s_usbSendDropFrames++;
        return 0U;
    }

    if (CDC_App_Write(data, len) == 0U)
    {
        s_usbSendDropFrames++;
        return 0U;
    }

    return 1U;
}

uint32_t USB_GetSendDropFrames(void)
{
    return s_usbSendDropFrames;
}

/* CRC16(Modbus 多项式 0xA001) */
uint16_t CRC16_Check(const uint8_t *data, uint16_t len)
{
    uint16_t crc16 = 0xFFFFU;
    uint16_t i;
    uint8_t  j;

    if ((data == 0) || (len == 0U))
    {
        return crc16;
    }

    for (i = 0U; i < len; i++)
    {
        crc16 ^= data[i];

        for (j = 0U; j < 8U; j++)
        {
            if ((crc16 & 0x0001U) != 0U)
            {
                crc16 >>= 1U;
                crc16 ^= 0xA001U;
            }
            else
            {
                crc16 >>= 1U;
            }
        }
    }

    return crc16;
}

/* 发送协议帧
 * 注意：
 * 1. 保持你原来的协议不变
 * 2. CRC 仍然按“高字节在前，低字节在后”发送
 */
uint8_t Send_Cmd_Data(uint8_t cmd, const uint8_t *datas, uint8_t len)
{
    uint8_t  buf[USB_FRAME_BUF_SIZE];
    uint16_t cnt = 0U;
    uint16_t i;
    uint16_t crc16;

    if ((datas == 0) && (len != 0U))
    {
        return 0U;
    }

    /* 总帧长 = 数据长度 + 7 */
    if (((uint16_t)len + USB_FRAME_OVERHEAD) > USB_FRAME_BUF_SIZE)
    {
        s_usbSendDropFrames++;
        return 0U;
    }

    buf[cnt++] = USB_FRAME_HEAD1;
    buf[cnt++] = USB_FRAME_HEAD2;
    buf[cnt++] = len;
    buf[cnt++] = cmd;

    for (i = 0U; i < (uint16_t)len; i++)
    {
        buf[cnt++] = datas[i];
    }

    /* 对 [帧头, 长度, 命令, 数据] 做 CRC */
    crc16 = CRC16_Check(buf, cnt);

    /* 保持你当前协议字节序：先高字节，后低字节 */
    buf[cnt++] = (uint8_t)(crc16 >> 8);
    buf[cnt++] = (uint8_t)(crc16 & 0xFFU);
    buf[cnt++] = USB_FRAME_TAIL;

    return Send(buf, cnt);
}


/*========================= 协议数据解析 =========================*/

void Data_Analysis(uint8_t cmd, const uint8_t* datas, uint8_t len)
{
    float x, y, z;

    switch (cmd)
    {
    case USB_CMD_ARM_SET_XYZ:
        /* 数据区格式：x(float) + y(float) + z(float) = 12字节 */
        if ((datas == 0) || (len != 12U))
        {
            uint8_t tx_data[2];
            tx_data[0] = ARM_IK_RESULT_PARAM_ERR;
            tx_data[1] = 0U;
            (void)Send_Cmd_Data(USB_CMD_ARM_IK_RESULT, tx_data, 2U);
            break;
        }

        x = USB_BytesToFloatLE(&datas[0]);
        y = USB_BytesToFloatLE(&datas[4]);
        z = USB_BytesToFloatLE(&datas[8]);

        /* 调用解算组件 */
        ArmIK_ComponentStep(x, y, z);
        break;

    default:
        /* 未知命令：忽略 */
        break;
    }
}


/*========================= 接收状态机 =========================*/

/* 单字节喂入解析器 */
void Receive(uint8_t bytedata)
{
    uint16_t calc_crc;

    switch (s_usbRxStep)
    {
    case 0: /* 等待帧头1 */
        if (bytedata == USB_FRAME_HEAD1)
        {
            USB_ParserRestartFromHead1();
        }
        break;

    case 1: /* 等待帧头2 */
        if (bytedata == USB_FRAME_HEAD2)
        {
            if (USB_ParserPushByte(bytedata) == 0U)
            {
                return;
            }
            s_usbRxStep = 2U;
        }
        else if (bytedata == USB_FRAME_HEAD1)
        {
            /* 连续 A5，把当前字节当成新的帧头1 */
            USB_ParserRestartFromHead1();
        }
        else
        {
            USB_ParserReset();
        }
        break;

    case 2: /* 接收长度 */
        s_usbRxLen = bytedata;

        /* 防止协议缓存溢出 */
        if (((uint16_t)s_usbRxLen + USB_FRAME_OVERHEAD) > USB_FRAME_BUF_SIZE)
        {
            USB_ParserReset();
            break;
        }

        if (USB_ParserPushByte(bytedata) == 0U)
        {
            return;
        }

        s_usbRxStep = 3U;
        break;

    case 3: /* 接收命令 */
        if (USB_ParserPushByte(bytedata) == 0U)
        {
            return;
        }

        s_usbRxCmd = bytedata;
        s_usbRxDataPtr = &usb_Buf[s_usbRxCnt];

        if (s_usbRxLen == 0U)
        {
            s_usbRxStep = 5U; /* 无数据，直接去收 CRC 高字节 */
        }
        else
        {
            s_usbRxStep = 4U;
        }
        break;

    case 4: /* 接收数据区 */
        if (USB_ParserPushByte(bytedata) == 0U)
        {
            return;
        }

        /* 当前已经收了多少个数据字节 = 总计数 - 头2 - 长度1 - 命令1 */
        if ((s_usbRxCnt - 4U) >= (uint16_t)s_usbRxLen)
        {
            s_usbRxStep = 5U;
        }
        break;

    case 5: /* 接收 CRC 高字节 */
        s_usbRxCrc16 = ((uint16_t)bytedata) << 8;
        s_usbRxStep = 6U;
        break;

    case 6: /* 接收 CRC 低字节 */
        s_usbRxCrc16 |= bytedata;

        calc_crc = CRC16_Check(usb_Buf, s_usbRxCnt);

        if (s_usbRxCrc16 == calc_crc)
        {
            s_usbRxStep = 7U;
        }
        else if (bytedata == USB_FRAME_HEAD1)
        {
            /* CRC 失败，但当前字节正好是 A5，则从新的帧头1继续 */
            USB_ParserRestartFromHead1();
        }
        else
        {
            USB_ParserReset();
        }
        break;

    case 7: /* 接收帧尾 */
        if (bytedata == USB_FRAME_TAIL)
        {
            Data_Analysis(s_usbRxCmd, s_usbRxDataPtr, s_usbRxLen);
            USB_ParserReset();
        }
        else if (bytedata == USB_FRAME_HEAD1)
        {
            /* 帧尾错，但这个字节可能是下一帧的头 */
            USB_ParserRestartFromHead1();
        }
        else
        {
            USB_ParserReset();
        }
        break;

    default:
        USB_ParserReset();
        break;
    }
}