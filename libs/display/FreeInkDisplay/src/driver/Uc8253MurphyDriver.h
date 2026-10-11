// WeAct 3.7-inch GDEY037T03 / UC8253 adaptation for CrossPoint 1.6.5.
// Enable ONLY in dv0_esp32s3_weact37: -DFREEINK_WEACT37_DRIVER=1
// Original Murphy implementation remains available when this flag is absent.
// WeAct controller sequence adapted from GxEPD2_370_GDEY037T03,
// by Jean-Marc Zingg (GxEPD2, GNU GPL v3). See LICENSE-GxEPD2.txt.
// Source: https://github.com/ZinggJM/GxEPD2/blob/master/src/gdey/GxEPD2_370_GDEY037T03.cpp
// Keeps the existing 416x240 facade -> 240x416 RAM rotation.
// Experimental: host protocol checks are not hardware validation.

#pragma once

// The Murphy notes below describe the fallback when the WeAct flag is absent.
// UC8253 panel driver — Murphy M3 (CrowPanel 3.7", 240x416 B/W, ESP32-S3).
// Ported from the community-sdk feat-support-for-m3 implementation.
//
// Distinct from the X3 UC8253 driver: the controller is 240x416 but the device is
// held portrait with controller RAM rotated 90° relative to the UI, so the facade
// owns a 416x240 framebuffer (BoardProfile.displayWidth/Height) and this driver
// rotates each plane when writing to the UC8253. OEM LUTs are loaded per refresh.
//
// Refresh: the controller is hardware-reset and re-initialised before every
// refresh (manufacturer guidance) so stale LUT/RAM state can't leave pixels
// half-latched. A full (GC) refresh then writes the new frame to BOTH planes so
// only WW/BB fire and every pixel is fully driven to target — clean. A FAST (DU)
// refresh is differential: previous frame -> DTM1 (old), new frame -> DTM2 (new),
// so unchanged pixels take WW/BB and changed pixels take the quick BW/WB transition
// kicks; the driver promotes a fast refresh to a full one every ghostClearInterval
// refreshes since DU ghosts over time. Without a previous frame (single-buffer
// builds) the fast path falls back to both-planes-new (WW/BB only).
//
// Selection: linked only when -DFREEINK_DRIVER_UC8253_MURPHY (Murphy board env).

#include "PanelDriver.h"

namespace freeink {

// One UC8253 waveform bank: the five LUT registers (0x20 VCOM .. 0x24 BB).
struct Uc8253MurphyLutBank {
  const uint8_t* vcom;  // 0x20
  const uint8_t* ww;    // 0x21
  const uint8_t* bw;    // 0x22
  const uint8_t* wb;    // 0x23
  const uint8_t* bb;    // 0x24
};

// Per-register LUT lengths. The UC8253 expects VCOM (0x20) and BW (0x22) at 56
// bytes and WW/WB/BB at 42 (manufacturer reference); each register is written at
// its own length so the controller latches the full waveform.
struct Uc8253MurphyLutLens {
  uint8_t vcom;  // 0x20
  uint8_t ww;    // 0x21
  uint8_t bw;    // 0x22
  uint8_t wb;    // 0x23
  uint8_t bb;    // 0x24
};

struct Uc8253MurphyConfig {
  Uc8253MurphyLutBank def;     // GC (ghost-clearing) waveforms
  Uc8253MurphyLutBank fast;    // DU (quick) waveforms
  Uc8253MurphyLutLens lens;    // per-register LUT byte counts
  uint16_t ghostClearInterval;  // Full cleanup after N fast refreshes; supports intervals above 255.
};

const Uc8253MurphyConfig& uc8253MurphyDefaultConfig();

class Uc8253MurphyDriver : public PanelDriver {
 public:
  explicit Uc8253MurphyDriver(const Uc8253MurphyConfig& cfg = uc8253MurphyDefaultConfig());

  uint32_t spiHz() const override;
  BusyPolarity busyPolarity() const override { return BusyPolarity::ActiveLow; }  // UC8253 BUSY low
  PanelGeometry geometry() const override;

  void begin(EpdBus& bus) override;
  void deepSleep(EpdBus& bus) override;
  void display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) override;

#if defined(FREEINK_WEACT37_DRIVER) && FREEINK_WEACT37_DRIVER
  // Protect the B/W base from unsupported legacy grayscale overlay calls.
  void displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut, bool factoryMode) override;
#endif

 private:
  void initController(EpdBus& bus);
  void loadLut(EpdBus& bus, const Uc8253MurphyLutBank& bank);
  void writePlane(EpdBus& bus, uint8_t command, const uint8_t* fb);  // rotates 416x240 fb -> 240x416 RAM
  void triggerRefresh(EpdBus& bus, bool turnOff);

  const Uc8253MurphyConfig& _cfg;

  uint16_t _fbW;   // framebuffer width  (416)
  uint16_t _fbH;   // framebuffer height (240)
  uint16_t _fbWb;  // framebuffer width bytes (52)

  bool _isScreenOn = false;
  uint16_t _fastRefreshCount = 0;
#if defined(FREEINK_WEACT37_DRIVER) && FREEINK_WEACT37_DRIVER
  bool _weactReady = false;
  bool _weactNeedsFull = true;
#endif
};

PanelDriver& uc8253MurphyDriver();

}  // namespace freeink
