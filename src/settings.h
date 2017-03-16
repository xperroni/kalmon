#ifndef KALMON_SETTINGS_H
#define KALMON_SETTINGS_H

namespace kalmon
{

struct Settings {
  double s2_P0;

  double s2_ax;

  double s2_ay;

  double s2_px;

  double s2_py;

  double s2_d;

  double s2_r;

  double s2_v;

  Settings();
};

Settings &getSettings();

} // namespace kalmon

#endif
