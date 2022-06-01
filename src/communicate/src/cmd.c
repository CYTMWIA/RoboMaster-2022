
#include "communicate/cmd.h"

#include <string.h>

const uint8_t kHeadingBytesCount = 2;
const uint8_t kHeadingBytes[] = {'I', 'R'};
const uint8_t kTailingBytesCount = 2;
const uint8_t kTailingBytes[] = {'O', 'N'};
const uint32_t kCmdToCvSize = kHeadingBytesCount + 1 + 16 + kTailingBytesCount;
const uint32_t kCmdToEcSize = kHeadingBytesCount + 8 + kTailingBytesCount;

/**
 * @brief 在内存中查找特定的数组
 *
 * @param mem
 * @param mem_size
 * @param target
 * @param target_size
 * @return int32_t -1表示查找失败；非负值表示首次出现的下标；
 */
int32_t find_mem(const uint8_t *mem, const size_t mem_size, const uint8_t *target,
                 const size_t target_size)
{
  int i, j;
  for (i = 0; i < mem_size - target_size + 1; i++)
  {
    for (j = 0; j < target_size; j++)
    {
      if (mem[i + j] != target[j])
        break;
      else if (j == target_size - 1)
        return i;
    }
  }
  return -1;
}

/*
 * CmdToCv 数组顺序：
 * __mode, bullet_speed, pitch, x_acc, y_acc
 *
 * uint8_t __mode; // 模式，用于压缩数据，具体内容详见下方注释
 * 该项仅在传输数据时使用，用户无需过多了解
 * 0 0 0 0 0 0 0 0
 *     └─┬─┘ └┬┘ └─【敌方颜色】
 *       │    └─【打击目标】
 *       └─【战术编号】
 */

void CmdToCv_make(const CmdToCv *cmd, uint8_t *data)
{
  uint8_t __mode = 0;
  __mode |= (cmd->enemy_color & 0x1) << 0;
  __mode |= (cmd->target & 0x11) << 1;
  __mode |= (cmd->tactic & 0x111) << 3;

  memcpy(data, kHeadingBytes, kHeadingBytesCount);
  data += kHeadingBytesCount;
  memcpy(data, &(__mode), 1);
  data += 1;
  memcpy(data, &(cmd->bullet_speed), 4);
  data += 4;
  memcpy(data, &(cmd->pitch), 4);
  data += 4;
  memcpy(data, &(cmd->x_acc), 4);
  data += 4;
  memcpy(data, &(cmd->y_acc), 4);
  data += 4;
  memcpy(data, kTailingBytes, kTailingBytesCount);
}

int32_t CmdToCv_parse(CmdToCv *cmd, uint8_t *data, uint32_t data_len)
{
  int idx = find_mem(data, data_len, kHeadingBytes, kHeadingBytesCount);
  if (idx == -1 || data_len - idx < kCmdToEcSize ||
      memcmp(data + idx + data_len - kTailingBytesCount, kTailingBytes, kTailingBytesCount) != 0)
    return -1;

  data = &(data[idx + kHeadingBytesCount]);

  cmd->enemy_color = (data[0] >> 0) & 1;
  cmd->target = (data[0] >> 1) & 3;
  cmd->tactic = (data[0] >> 3) & 7;
  data += 1;

  memcpy(&(cmd->bullet_speed), data, 4);
  data += 4;
  memcpy(&(cmd->pitch), data, 4);
  data += 4;
  memcpy(&(cmd->x_acc), data, 4);
  data += 4;
  memcpy(&(cmd->y_acc), data, 4);

  return 0;
}

/*
 * CmdToEc 数组顺序：
 * pitch, yaw
 */

void CmdToEc_make(const CmdToEc *cmd, uint8_t *data)
{
  memcpy(data, kHeadingBytes, kHeadingBytesCount);
  data += kHeadingBytesCount;
  memcpy(data, &(cmd->pitch), 4);
  data += 4;
  memcpy(data, &(cmd->yaw), 4);
  data += 4;
  memcpy(data, kTailingBytes, kTailingBytesCount);
}

int32_t CmdToEc_parse(CmdToEc *cmd, uint8_t *data, uint32_t data_len)
{
  int idx = find_mem(data, data_len, kHeadingBytes, kHeadingBytesCount);
  if (idx == -1 || data_len - idx < kCmdToEcSize ||
      memcmp(data + data_len - kTailingBytesCount, kTailingBytes, kTailingBytesCount) != 0)
    return -1;

  data = &(data[idx + kHeadingBytesCount]);

  memcpy(&(cmd->pitch), data, 4);
  data += 4;
  memcpy(&(cmd->yaw), data, 4);

  return 0;
}
