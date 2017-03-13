#include "settings.h"

namespace kalmon
{

Settings::Settings():
  s_P0(0.01),
  s_Qx(0.01),
  s_Qy(0.01),
  s_px(0.01),
  s_py(0.01),
  s_r(0.01),
  s_d(0.01),
  s_v(0.01)
{
}

Settings &getSettings()
{
  static Settings settings;

  return settings;
}

} // namespace kalmon
