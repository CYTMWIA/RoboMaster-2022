
#include "communicate/cmd.h"

#include <string.h>

#define __PUSH(data_ptr, len)  \
  memcpy(data, data_ptr, len); \
  data += len;
#define PUSH_HEADING_BYTES() __PUSH(CMD_HEADING_BYTES, CMD_HEADING_BYTES_LENGTH)
#define PUSH_TAILING_BYTES() __PUSH(CMD_TAILING_BYTES, CMD_TAILING_BYTES_LENGTH)
#define PUSH(pod_lv) __PUSH(&(pod_lv), sizeof(pod_lv));

#define PULL_HEADING_BYTES() idx = (idx + CMD_HEADING_BYTES_LENGTH) % data_len;
#define PULL(pod_lv)                               \
  if (idx + sizeof(pod_lv) <= data_len)            \
  {                                                \
    memcpy(&(pod_lv), data + idx, sizeof(pod_lv)); \
    idx = (idx + sizeof(pod_lv)) % data_len;       \
  }                                                \
  else                                             \
  {                                                \
    for (int i = 0; i < (int)sizeof(pod_lv); i++)  \
    {                                              \
      memcpy(&(pod_lv) + i, data + idx, 1);        \
      idx = (idx + 1) % data_len;                  \
    }                                              \
  }

/*
 * CmdToCv 数组顺序与声明顺序相同
 */
void CmdToCv_make(const CmdToCv *cmd, uint8_t *data)
{
  PUSH_HEADING_BYTES();
  PUSH(cmd->enemy_color);
  PUSH(cmd->target);
  PUSH(cmd->bullet_speed);
  PUSH(cmd->pitch);
  PUSH(cmd->yaw);
  PUSH_TAILING_BYTES();
}

int32_t CmdToCv_parse(CmdToCv *cmd, uint8_t *data, uint32_t data_len)
{
  int idx = data_len - 1;
  while (idx > -1)
  {
    int head_eq_flag = 1;
    for (int i = 0; i < CMD_HEADING_BYTES_LENGTH; i++)
      if (CMD_HEADING_BYTES[i] != data[(idx + i) % data_len])
      {
        head_eq_flag = 0;
        break;
      }

    if (head_eq_flag &&
        memcmp(data + (idx + CMD_HEADING_BYTES_LENGTH + CMD_TO_CV_CONTENT_LENGTH) % data_len,
               CMD_TAILING_BYTES, CMD_TAILING_BYTES_LENGTH) == 0)
    {
      PULL_HEADING_BYTES();
      PULL(cmd->enemy_color);
      PULL(cmd->target);
      PULL(cmd->bullet_speed);
      PULL(cmd->pitch);
      PULL(cmd->yaw);
      return 0;
    }
    idx--;
  }

  return -1;
}

/*
 * CmdToEc 数组顺序与声明顺序相同
 */

void CmdToEc_make(const CmdToEc *cmd, uint8_t *data)
{
  PUSH_HEADING_BYTES();
  PUSH(cmd->pitch);
  PUSH(cmd->yaw);
  PUSH(cmd->fire);
  PUSH_TAILING_BYTES();
}

int32_t CmdToEc_parse(CmdToEc *cmd, uint8_t *data, uint32_t data_len)
{
  int idx = data_len - 1;
  while (idx > -1)
  {
    int head_eq_flag = 1;
    for (int i = 0; i < CMD_HEADING_BYTES_LENGTH; i++)
      if (CMD_HEADING_BYTES[i] != data[(idx + i) % data_len])
      {
        head_eq_flag = 0;
        break;
      }

    if (head_eq_flag &&
        memcmp(data + (idx + CMD_HEADING_BYTES_LENGTH + CMD_TO_EC_CONTENT_LENGTH) % data_len,
               CMD_TAILING_BYTES, CMD_TAILING_BYTES_LENGTH) == 0)
    {
      PULL_HEADING_BYTES();
      PULL(cmd->pitch);
      PULL(cmd->yaw);
      PULL(cmd->fire);
      return 0;
    }
    idx--;
  }

  return -1;
}

#undef __PUSH
#undef PUSH_HEADING_BYTES
#undef PUSH_TAILING_BYTES
#undef PUSH
#undef PULL_HEADING_BYTES
#undef PULL
