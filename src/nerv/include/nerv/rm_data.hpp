#ifndef __NERV_RM_DATA_HPP__
#define __NERV_RM_DATA_HPP__

/*
 * 统一单位
 * 长度：毫米 mm
 * 时间：秒 s
 * 质量：克 g
 */

namespace nerv::rm_data
{
enum BulletType
{
  BULLET_17,
  BULLET_42
};

const double LARGE_ARMOR_HEIGHT = 127;  // 大装甲板 高度
const double LARGE_ARMOR_WIDTH = 230;   // 大装甲板 宽度

const double SMALL_ARMOR_HEIGHT = 125;  // 小装甲板 高度
const double SMALL_ARMOR_WIDTH = 135;   // 小装甲板 宽度

const double LIGHTBAR_HEIGHT = 55;  // 灯条高度（长边）

const double BULLET_17_DIAMETER = 17;  // 小弹丸直径
const double BULLET_17_MASS = 3.2;     // 小弹丸质量
const double BULLET_42_DIAMETER = 42;  // 大弹丸直径
const double BULLET_42_MASS = 41;      // 大弹丸质量
}  // namespace nerv::rm_data

#endif  // __RM_DATA_HPP__