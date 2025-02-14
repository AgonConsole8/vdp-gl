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



#include <alloca.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "soc/rtc.h"
#include "esp_spi_flash.h"
#include "esp_heap_caps.h"

#include "fabutils.h"
#include "vgapalettedcontroller.h"
#include "devdrivers/swgenerator.h"



#pragma GCC optimize ("O2")



namespace fabgl {





/*************************************************************************************/
/* VGAPalettedController definitions */


VGAPalettedController::VGAPalettedController(int linesCount, int columnsQuantum, NativePixelFormat nativePixelFormat, int viewPortRatioDiv, int viewPortRatioMul, intr_handler_t isrHandler, int signalTableSize)
  : m_columnsQuantum(columnsQuantum),
    m_nativePixelFormat(nativePixelFormat),
    m_viewPortRatioDiv(viewPortRatioDiv),
    m_viewPortRatioMul(viewPortRatioMul),
    m_isrHandler(isrHandler),
    m_signalTableSize(signalTableSize)
{
  m_linesCount = linesCount;
  m_lines   = (volatile uint8_t**) heap_caps_malloc(sizeof(uint8_t*) * m_linesCount, MALLOC_CAP_32BIT | MALLOC_CAP_INTERNAL);
  m_palette = (RGB222*) heap_caps_malloc(sizeof(RGB222) * getPaletteSize(), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  createPalette(0);
  uint16_t signalList[2] = { 0, 0 };
  m_signalList = createSignalList(signalList, 1);
  m_currentSignalItem = m_signalList;
}


VGAPalettedController::~VGAPalettedController()
{
  heap_caps_free(m_palette);
  heap_caps_free(m_lines);
  for (auto it = m_signalMaps.begin(); it != m_signalMaps.end();) {
    if (it->second) {
      heap_caps_free((void *)it->second);
    }
    it = m_signalMaps.erase(it);
  }
  deleteSignalList(m_signalList);
}


void VGAPalettedController::init()
{
  VGABaseController::init();

  m_doubleBufferOverDMA      = false;
}


void VGAPalettedController::end()
{
  VGABaseController::end();
}


void VGAPalettedController::suspendBackgroundPrimitiveExecution()
{
  VGABaseController::suspendBackgroundPrimitiveExecution();
}

// make sure view port height is divisible by m_linesCount, view port width is divisible by m_columnsQuantum
void VGAPalettedController::checkViewPortSize()
{
  m_viewPortHeight &= ~(m_linesCount - 1);
  m_viewPortWidth  &= ~(m_columnsQuantum - 1);
}


void VGAPalettedController::allocateViewPort()
{
  VGABaseController::allocateViewPort(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL, m_viewPortWidth / m_viewPortRatioDiv * m_viewPortRatioMul);

  for (int i = 0; i < m_linesCount; ++i)
    m_lines[i] = (uint8_t*) heap_caps_malloc(m_viewPortWidth, MALLOC_CAP_DMA);
}


void VGAPalettedController::freeViewPort()
{
  VGABaseController::freeViewPort();

  for (int i = 0; i < m_linesCount; ++i) {
    heap_caps_free((void*)m_lines[i]);
    m_lines[i] = nullptr;
  }
}


void VGAPalettedController::setResolution(VGATimings const& timings, int viewPortWidth, int viewPortHeight, bool doubleBuffered)
{
  VGABaseController::setResolution(timings, viewPortWidth, viewPortHeight, doubleBuffered);

  s_viewPort        = m_viewPort;
  s_viewPortVisible = m_viewPortVisible;

  // fill view port
  for (int i = 0; i < m_viewPortHeight; ++i)
    memset((void*)(m_viewPort[i]), nativePixelFormat() == NativePixelFormat::SBGR2222 ? m_HVSync : 0, m_viewPortWidth / m_viewPortRatioDiv * m_viewPortRatioMul);

  setupDefaultPalette();
  updateRGB2PaletteLUT();

  calculateAvailableCyclesForDrawings();

  // must be started before interrupt alloc
  startGPIOStream();

  // ESP_INTR_FLAG_LEVEL1: should be less than PS2Controller interrupt level, necessary when running on the same core
  if (m_isr_handle == nullptr) {
    CoreUsage::setBusiestCore(FABGLIB_VIDEO_CPUINTENSIVE_TASKS_CORE);
    esp_intr_alloc_pinnedToCore(ETS_I2S1_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM, m_isrHandler, this, &m_isr_handle, FABGLIB_VIDEO_CPUINTENSIVE_TASKS_CORE);
    I2S1.int_clr.val     = 0xFFFFFFFF;
    I2S1.int_ena.out_eof = 1;
  }

  resumeBackgroundPrimitiveExecution();
}


void VGAPalettedController::onSetupDMABuffer(lldesc_t volatile * buffer, bool isStartOfVertFrontPorch, int scan, bool isVisible, int visibleRow)
{
  if (isVisible) {
    buffer->buf = (uint8_t *) m_lines[visibleRow % m_linesCount];

    // generate interrupt every half m_linesCount
    if ((scan == 0 && (visibleRow % (m_linesCount / 2)) == 0)) {
      if (visibleRow == 0)
        s_frameResetDesc = buffer;
      buffer->eof = 1;
    }
  }
}


int VGAPalettedController::getPaletteSize()
{
  switch (nativePixelFormat()) {
    case NativePixelFormat::PALETTE2:
      return 2;
    case NativePixelFormat::PALETTE4:
      return 4;
    case NativePixelFormat::PALETTE8:
      return 8;
    case NativePixelFormat::PALETTE16:
      return 16;
    default:
      return 0;
  }
}

void VGAPalettedController::setPaletteItem(int index, RGB888 const & color)
{
  setItemInPalette(0, index, color);
}


void VGAPalettedController::setItemInPalette(uint16_t paletteId, int index, RGB888 const & color)
{
  if (m_signalMaps.find(paletteId) == m_signalMaps.end()) {
    if (!createPalette(paletteId)) {
      return;
    }
  }
  index %= getPaletteSize();
  if (paletteId == 0) {
    m_palette[index] = color;
  }
  auto packed222 = RGB888toPackedRGB222(color);
  packSignals(index, packed222, m_signalMaps[paletteId]);
}



// rebuild m_packedRGB222_to_PaletteIndex
void VGAPalettedController::updateRGB2PaletteLUT()
{
  auto paletteSize = getPaletteSize();
  for (int r = 0; r < 4; ++r)
    for (int g = 0; g < 4; ++g)
      for (int b = 0; b < 4; ++b) {
        double H1, S1, V1;
        rgb222_to_hsv(r, g, b, &H1, &S1, &V1);
        int bestIdx = 0;
        int bestDst = 1000000000;
        for (int i = 0; i < paletteSize; ++i) {
          double H2, S2, V2;
          rgb222_to_hsv(m_palette[i].R, m_palette[i].G, m_palette[i].B, &H2, &S2, &V2);
          double AH = H1 - H2;
          double AS = S1 - S2;
          double AV = V1 - V2;
          int dst = AH * AH + AS * AS + AV * AV;
          if (dst <= bestDst) {  // "<=" to prioritize higher indexes
            bestIdx = i;
            bestDst = dst;
            if (bestDst == 0)
              break;
          }
        }
        m_packedRGB222_to_PaletteIndex[r | (g << 2) | (b << 4)] = bestIdx;
      }
}


bool VGAPalettedController::createPalette(uint16_t paletteId)
{
  if (m_signalTableSize == 0) {
    return false;
  }
  if (m_signalMaps.find(paletteId) == m_signalMaps.end()) {
    m_signalMaps[paletteId] = heap_caps_malloc(m_signalTableSize, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!m_signalMaps[paletteId]) {
      m_signalMaps.erase(paletteId);
      // create failed
      return false;
    }
    if (paletteId == 0) {
      return true;
    }
  }
  // Duplicate palette 0 into new palette
  if (paletteId != 0) {
    memcpy(m_signalMaps[paletteId], m_signalMaps[0], m_signalTableSize);
  }
  return true;
}


void VGAPalettedController::deletePalette(uint16_t paletteId)
{
  if (paletteId == 0) {
    return;
  }
  if (m_signalMaps.find(paletteId) != m_signalMaps.end()) {
    auto signals = m_signalMaps[paletteId];
    auto signalItem = m_signalList;
    while (signalItem) {
      if (signalItem->signals == signals) {
        signalItem->signals = m_signalMaps[0];
      }
      signalItem = signalItem->next;
    }
    heap_caps_free(m_signalMaps[paletteId]);
    m_signalMaps.erase(paletteId);
  }
}


void VGAPalettedController::deleteSignalList(PaletteListItem * item)
{
  if (item) {
    deleteSignalList(item->next);
    heap_caps_free(item);
  }
}


void VGAPalettedController::updateSignalList(uint16_t * rawList, int entries)
{
  // Walk list, updating existing signal list
  // creating new list if we exceed the current list,
  // deleting any remaining items if we have fewer entries
  PaletteListItem * item = m_signalList;
  int row = 0;

  while (entries) {
    auto rows = rawList[0];
    auto paletteID = rawList[1];
    rawList += 2;

    row += rows;
    item->endRow = row;
    if (m_signalMaps.find(paletteID) != m_signalMaps.end()) {
      item->signals = m_signalMaps[paletteID];
    } else {
      item->signals = m_signalMaps[0];
    }

    entries--;

    if (entries) {
      if (item->next) {
        item = item->next;
      } else {
        item->next = createSignalList(rawList, entries, row);
        return;
      }
    }
  }

  if (item->next) {
    deleteSignalList(item->next);
    item->next = NULL;
  }
}


PaletteListItem * VGAPalettedController::createSignalList(uint16_t * rawList, int entries, int row)
{
  PaletteListItem * item = (PaletteListItem *) heap_caps_malloc(sizeof(PaletteListItem), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  auto rows = rawList[0];
  auto paletteID = rawList[1];

  item->endRow = row + rows;

  if (m_signalMaps.find(paletteID) != m_signalMaps.end()) {
    item->signals = m_signalMaps[paletteID];
  } else {
    item->signals = m_signalMaps[0];
  }

  if (entries > 1) {
    item->next = createSignalList(rawList + 2, entries - 1, row + rows);
  } else {
    item->next = NULL;
  }

  return item;
}


void * IRAM_ATTR VGAPalettedController::getSignalsForScanline(int scanLine) {
  if (scanLine < m_currentSignalItem->endRow) {
    return m_currentSignalItem->signals;
  }
  while (m_currentSignalItem->next && (scanLine >= m_currentSignalItem->endRow)) {
    m_currentSignalItem = m_currentSignalItem->next;
  }
  return m_currentSignalItem->signals;
}


void VGAPalettedController::swapBuffers()
{
  VGABaseController::swapBuffers();
  s_viewPort        = m_viewPort;
  s_viewPortVisible = m_viewPortVisible;
}



} // end of namespace

