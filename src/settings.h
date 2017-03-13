#ifndef KALMON_SETTINGS_H
#define KALMON_SETTINGS_H

namespace kalmon
{

struct Settings {
  double s_P0;

  double s_Qx;

  double s_Qy;

  double s_px;

  double s_py;

  double s_d;

  double s_r;

  double s_v;

  Settings();
};

Settings &getSettings();

} // namespace kalmon

#endif
