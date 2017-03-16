#include "settings.h"

namespace kalmon
{

Settings::Settings():
  s2_P0(0.01),
  s2_ax(10),
  s2_ay(8),
  s2_px(0.0225),
  s2_py(0.02),
  s2_r(0.1),
  s2_d(0.1),
  s2_v(0.1)
{
  // Nothing to do.
}

Settings &getSettings()
{
  static Settings settings;

  return settings;
}

} // namespace kalmon
