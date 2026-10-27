/*
 *  radxa_led_driver.cpp
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#include "explorer_leds/radxa_led_driver.h"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <utility>

#include "explorer_leds/sk6812_encoder.h"

namespace explorer_leds
{
RadxaLedDriver::RadxaLedDriver(LedStripConfig config) : config_(std::move(config)) {}

RadxaLedDriver::~RadxaLedDriver() { close(); }

std::optional<std::string> RadxaLedDriver::open()
{
  close();

  fd_ = ::open(config_.spi_device.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd_ < 0)
  {
    return "Cannot open LED SPI device " + config_.spi_device + ": " + std::strerror(errno);
  }

  std::uint8_t mode = SPI_MODE_0;
  std::uint32_t speed_hz = SK6812_SPI_SPEED_HZ;
  if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0)
  {
    const std::string error = std::strerror(errno);
    close();
    return "Cannot configure LED SPI device " + config_.spi_device + ": " + error;
  }
  return std::nullopt;
}

void RadxaLedDriver::close()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

bool RadxaLedDriver::write(const std::vector<Rgbw>& frame)
{
  std::vector<Rgbw> pixels = frame;
  pixels.resize(config_.led_count);
  return write_bytes_(encode_sk6812_frame(pixels, config_.rgbw, config_.max_brightness));
}

bool RadxaLedDriver::write_bytes_(const std::vector<std::uint8_t>& bytes) const
{
  if (fd_ < 0)
  {
    return false;
  }
  // One write is one SPI transfer: the strip needs its frame without gaps
  const ssize_t written = ::write(fd_, bytes.data(), bytes.size());
  return written == static_cast<ssize_t>(bytes.size());
}
}  // namespace explorer_leds
