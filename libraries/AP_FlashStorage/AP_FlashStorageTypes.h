/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  the AP_FlashStorage types, split out of AP_FlashStorage.h so that a board
  header can select one by name rather than by number.

  This header must stay free of includes: the board headers are processed from
  AP_HAL_Boards.h, which AP_FlashStorage.h itself only reaches later through
  AP_HAL.h, so nothing from there is available to them.
 */
#pragma once

/*
  we support several different types of flash which have different restrictions
 */
#define AP_FLASHSTORAGE_TYPE_F1     1 // F1 and F3
#define AP_FLASHSTORAGE_TYPE_F4     2 // F4 and F7
#define AP_FLASHSTORAGE_TYPE_H7     3 // H7
#define AP_FLASHSTORAGE_TYPE_G4     4 // G4
#define AP_FLASHSTORAGE_TYPE_RP2350 5 // RP2350
