/*
 * Copyright (c) Helio Perroni Filho <xperroni@gmail.com>
 *
 * This file is part of KalmOn.
 *
 * KalmOn is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * KalmOn is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with KalmOn. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef KALMON_SETTINGS_H
#define KALMON_SETTINGS_H

namespace kalmon {

/**
 * @brief Record of system settings.
 */
struct Settings {
  /** @brief Initial state variance. */
  double s2_P0;

  /** @brief Process noise variance across the X axis. */
  double s2_ax;

  /** @brief Process noise variance across the Y axis. */
  double s2_ay;

  /** @brief Laser measurement position variance across the X axis. */
  double s2_px;

  /** @brief Laser measurement position variance across the Y axis. */
  double s2_py;

  /** @brief Radar position variance. */
  double s2_d;

  /** @brief Radar direction variance. */
  double s2_r;

  /** @brief Radar speed variance. */
  double s2_v;

  /**
   * @brief Default constructor.
   */
  Settings();
};

/**
 * @brief Retrieve the record of system settings.
 */
Settings &getSettings();

} // namespace kalmon

#endif
