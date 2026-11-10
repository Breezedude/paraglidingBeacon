#include "battery.h"

namespace battery {
namespace {

// Open circuit voltage in millivolts at 100, 90, ... 0 percent, for the
// 700 mAh cell in this board.
constexpr uint16_t kCurve[11] = {4190, 4042, 3957, 3885, 3820, 3776,
                                 3746, 3725, 3696, 3644, 3100};

}  // namespace

uint8_t percentFromMillivolts(uint16_t mv) {
  if (mv >= kCurve[0])  return 100;
  if (mv <= kCurve[10]) return 0;

  for (int i = 1; i <= 10; ++i) {
    if (mv >= kCurve[i]) {
      const uint16_t upper = kCurve[i - 1];
      const uint16_t lower = kCurve[i];
      const uint32_t part  = uint32_t(mv - lower) * 10U / uint32_t(upper - lower);
      return uint8_t((10 - i) * 10 + part);
    }
  }
  return 0;
}

}  // namespace battery
