/*
  Created by Fabrizio Di Vittorio (fdivitto2013@gmail.com) - <http://www.fabgl.com>
  Copyright (c) 2019-2022 Fabrizio Di Vittorio.
  All rights reserved.


* Please contact fdivitto2013@gmail.com if you need a commercial license.


* This library and related software is available under GPL v3.

  FabGL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  FabGL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with FabGL.  If not, see <http://www.gnu.org/licenses/>.
 */


#pragma once



/**
 * @file
 *
 * @brief This file contains fabgl::VGAPalettedController definition.
 */


#include <stdint.h>
#include <stddef.h>
#include <atomic>
#include <unordered_map>

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "fabglconf.h"
#include "fabutils.h"
#include "devdrivers/swgenerator.h"
#include "displaycontroller.h"
#include "dispdrivers/vgabasecontroller.h"




namespace fabgl {






/**
* @brief Represents the base class for paletted bitmapped controllers like VGA16Controller, VGA8Controller, etc..
*/
class VGAPalettedController : public VGABaseController {

public:

  VGAPalettedController(int linesCount, int columnsQuantum, NativePixelFormat nativePixelFormat, int viewPortRatioDiv, int viewPortRatioMul, intr_handler_t isrHandler, int signalTableSize = 0);
  ~VGAPalettedController();

  // unwanted methods
  VGAPalettedController(VGAPalettedController const&) = delete;
  void operator=(VGAPalettedController const&)        = delete;

  void end();

  // abstract method of BitmappedDisplayController
  void suspendBackgroundPrimitiveExecution();

  // import "modeline" version of setResolution
  using VGABaseController::setResolution;

  void setResolution(VGATimings const& timings, int viewPortWidth = -1, int viewPortHeight = -1, bool doubleBuffered = false);

  int getPaletteSize();

  virtual int colorsCount()      { return getPaletteSize(); }

  /**
   * @brief Determines the maximum time allowed to process primitives
   *
   * Primitives processing is always started at the beginning of vertical blank.
   * Unfortunately this time is limited and not all primitive may be processed, so processing all primitives may required more frames.
   * This method expands the allowed time to half of a frame. This increase drawing speed but may show some flickering.
   *
   * The default is False (fast drawings, possible flickering).
   *
   * @param value True = allowed time to process primitives is limited to the vertical blank. Slow, but avoid flickering. False = allowed time is the half of an entire frame. Fast, but may flick.
   */
  void setProcessPrimitivesOnBlank(bool value)          { m_processPrimitivesOnBlank = value; }

  // returns "static" version of m_viewPort
  static uint8_t * sgetScanline(int y)                  { return (uint8_t*) s_viewPort[y]; }

  // abstract method of BitmappedDisplayController
  NativePixelFormat nativePixelFormat()                 { return m_nativePixelFormat; }

  // Should be called after the palette is updated.
  void updateRGB2PaletteLUT();

  /**
   * @brief Creates a new palette (signal block) with a 16-bit ID, copying the default palette
   * 
   * @param paletteId ID of the new palette
   */
  bool createPalette(uint16_t paletteId);


  /**
   * @brief Deletes a palette (signal block) with a 16-bit ID
   */
  void deletePalette(uint16_t paletteId);

  /**
   * @brief Sets color of specified palette item in default palette
   *
   * @param index Palette item (0..<palette size>)
   * @param color Color to assign to this item
   *
   * Example:
   *
   *     // Color item 0 is pure Red
   *     displayController.setPaletteItem(0, RGB888(255, 0, 0));
   */
  void setPaletteItem(int index, RGB888 const & color);

  /**
   * @brief Sets color of specified palette item in a given palette
   * 
   * @param paletteId ID of the palette
   * @param index Index of the item in the palette
   * @param color Color to assign to this item
   */
  void setItemInPalette(uint16_t paletteId, int index, RGB888 const & color);


  /**
   * @brief Creates a new signal list based off simple pairs of row count and palette ID
   * 
   * @param rawList List of row count and palette ID pairs
   * @param entries Number of entries in the list
   */
  void updateSignalList(uint16_t * rawList, int entries);



protected:

  void init();

  virtual void setupDefaultPalette() = 0;

  uint8_t RGB888toPaletteIndex(RGB888 const & rgb) {
    return m_packedRGB222_to_PaletteIndex[RGB888toPackedRGB222(rgb)];
  }

  uint8_t RGB2222toPaletteIndex(uint8_t value) {
    return m_packedRGB222_to_PaletteIndex[value & 0b00111111];
  }

  uint8_t RGB8888toPaletteIndex(RGBA8888 value) {
    return RGB888toPaletteIndex(RGB888(value.R, value.G, value.B));
  }

  // abstract method of BitmappedDisplayController
  void swapBuffers();

  virtual void packSignals(int index, uint8_t packed222, void * signals) = 0;

  void * getSignalsForScanline(int scanline);


  RGB222 *                    m_palette;

  int                         m_signalTableSize;

  // signal maps for mapping framebuffer data into signals for a given palette ID
  std::unordered_map<uint16_t, void *>     m_signalMaps;

  PaletteListItem *           m_signalList;
  PaletteListItem *           m_currentSignalItem;


private:

  void allocateViewPort();
  void freeViewPort();
  void checkViewPortSize();
  void onSetupDMABuffer(lldesc_t volatile * buffer, bool isStartOfVertFrontPorch, int scan, bool isVisible, int visibleRow);

  PaletteListItem * createSignalList(uint16_t * rawList, int entries, int row = 0);
  void deleteSignalList(PaletteListItem * item);

  uint8_t                     m_packedRGB222_to_PaletteIndex[64];

  // configuration
  int                         m_columnsQuantum; // viewport width must be divisble by m_columnsQuantum
  NativePixelFormat           m_nativePixelFormat;
  int                         m_viewPortRatioDiv;
  int                         m_viewPortRatioMul;
  intr_handler_t              m_isrHandler;


};



} // end of namespace








