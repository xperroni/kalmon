#include "settings.h"

namespace kalmon
{

Settings::Settings():
  s_P0(0.01),
  s_Qx(10),
  s_Qy(8),
  s_px(0.0225),
  s_py(0.02),
  s_r(0.1),
  s_d(0.1),
  s_v(0.1)
{
  // Nothing to do.
}

Settings &getSettings()
{
  static Settings settings;

  return settings;
}

} // namespace kalmon
