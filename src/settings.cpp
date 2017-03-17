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

#include "settings.h"

namespace kalmon {

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

Settings &getSettings() {
  static Settings settings;

  return settings;
}

} // namespace kalmon
