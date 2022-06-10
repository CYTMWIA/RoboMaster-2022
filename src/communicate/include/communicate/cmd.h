#ifndef __CVEC_CMD_H__
#define __CVEC_CMD_H__

#include <stdint.h>

/* ！！！注意！！！
 * 通信中，所有角度单位都是角度（angle），而非弧度！
 */

#define CMD_HEADING_BYTES ("IR")
#define CMD_HEADING_BYTES_LENGTH (2)
#define CMD_TAILING_BYTES ("ON")
#define CMD_TAILING_BYTES_LENGTH (2)
#define CMD_TO_CV_CONTENT_LENGTH (14)
#define CMD_TO_CV_SIZE \
  (CMD_HEADING_BYTES_LENGTH + CMD_TO_CV_CONTENT_LENGTH + CMD_TAILING_BYTES_LENGTH)
#define CMD_TO_EC_CONTENT_LENGTH (9)
#define CMD_TO_EC_SIZE \
  (CMD_HEADING_BYTES_LENGTH + CMD_TO_EC_CONTENT_LENGTH + CMD_TAILING_BYTES_LENGTH)

typedef enum
{
  kEnemyColor_Blue = 0,
  kEnemyColor_Red = 1
} EnemyColor;

typedef enum
{
  kAimTarget_None = 0,
  kAimTarget_Armor = 1,
  kAimTarget_SmallBuff = 2,
  kAimTarget_BigBuff = 3
} AimTarget;

/**
 * @brief 发送给 视觉 的数据结构
 *
 */
typedef struct CmdToCv
{
  EnemyColor enemy_color;  // 【敌方颜色】0-蓝 1-红
  AimTarget target;  // 【打击目标】00-关闭自瞄 01-装甲板 10-小能量机关 11-大能量机关
  float bullet_speed;  // 子弹初速度（m/s）
  float pitch;         // 当前pitch轴角度（水平为零，向上为正）
  float yaw;           // 当前yaw轴角度（正前为零，向左为正）
} CmdToCv;

/**
 * @brief 构造传输 CmdToCv 所用的数组
 *
 * @param cmd
 * @param data 数组，长度至少为 kCmdToCvSize
 */
void CmdToCv_make(const CmdToCv *cmd, uint8_t *data);

/**
 * @brief 解析传输 CmdToCv 所用的数组
 *
 * @param cmd
 * @param data
 * @param data_len 数组长度
 * @return int32_t 负数表示解析失败，0表示解析成功
 */
int32_t CmdToCv_parse(CmdToCv *cmd, uint8_t *data, uint32_t data_len);

/**
 * @brief 发送给 电控 的数据结构
 *
 */
typedef struct CmdToEc
{
  float pitch;   // pitch轴偏差（当前pitch+偏差pitch=目标pitch）
  float yaw;     // yaw轴偏差，与pitch轴同理
  uint8_t fire;  // 是否推荐发射
} CmdToEc;

/**
 * @brief 构造传输 CmdToEc 所用的数组
 *
 * @param cmd
 * @param data 数组，长度至少为 kCmdToEcSize
 */
void CmdToEc_make(const CmdToEc *cmd, uint8_t *data);

/**
 * @brief 解析传输 CmdToEc 所用的数组
 *
 * @param cmd
 * @param data
 * @param data_len 数组长度
 * @return int32_t 负数表示解析失败，0表示解析成功
 */
int32_t CmdToEc_parse(CmdToEc *cmd, uint8_t *data, uint32_t data_len);

typedef CmdToCv RobotStatus;  // 机器人状态
typedef CmdToEc CvStatus;     // 视觉计算 状态/结果

#endif