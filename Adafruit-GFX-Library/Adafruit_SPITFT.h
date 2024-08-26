/*!
 * @file Adafruit_SPITFT.h
 *
 * Part of Adafruit's GFX graphics library. Originally this class was
 * written to handle a range of color TFT displays connected via SPI,
 * but over time this library and some display-specific subclasses have
 * mutated to include some color OLEDs as well as parallel-interfaced
 * displays. The name's been kept for the sake of older code.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor "ladyada" Fried for Adafruit Industries,
 * with contributions from the open source community.
 *
 * BSD license, all text here must be included in any redistribution.
 */
// Updated to work with nRF5 SDK in 2020 by Adam Green (https://github.com/adamgreen)

#ifndef _ADAFRUIT_SPITFT_H_
#define _ADAFRUIT_SPITFT_H_

#include "Adafruit_GFX.h"
#include <stdint.h>
#include <nrf_drv_spi.h>
#include <nrf_gpio.h>
#include <nrf_assert.h>


// The SPI data is queued up in this circular buffer so that small operations can be queued up here from the main
// thread and then pulled out and transmitted later from an ISR.
template <uint32_t BUFFER_SIZE>
class CircularBuffer {
public:
    CircularBuffer()
    {
        m_read = 0;
        m_write = 0;
        m_state = None;
        // BUFFER_SIZE must be a power of 2.
        ASSERT ( 0 == (BUFFER_SIZE & (BUFFER_SIZE - 1)) );
    }

    bool isEmpty()
    {
        return m_read == m_write;
    }

    bool isFull()
    {
        return wrapIndex(m_write + 1) == m_read;
    }

    void start()
    {
        m_state = Start;
    }

    void writeCommand(uint8_t command)
    {
        writeByteAndFlag(command, m_state | Command);
    }

    void writeData(uint8_t data)
    {
        writeByteAndFlag(data, m_state);
    }

    void writeData(uint16_t data)
    {
        writeByteAndFlag((data >> 8) & 0xFF, m_state);
        writeByteAndFlag(data & 0xFF, m_state);
    }

    void writeData(uint32_t data)
    {
        writeByteAndFlag((data >> 24) & 0xFF, m_state);
        writeByteAndFlag((data >> 16) & 0xFF, m_state);
        writeByteAndFlag((data >> 8) & 0xFF, m_state);
        writeByteAndFlag(data & 0xFF, m_state);
    }

    void end()
    {
        writeByteAndFlag(0x00, End);
    }

    // Returns array of bytes that have the same flags set.
    bool readBytes(uint8_t** ppBytes, uint32_t* pLen, uint8_t* pFlags)
    {
        if (isEmpty())
        {
            return false;
        }
        *ppBytes = &m_bytes[m_read];
        uint8_t  flags = m_flags[m_read];
        uint32_t len = 1;
        m_read = wrapIndex(m_read + 1);
        while (!isEmpty() && m_flags[m_read] == flags && m_read > 0) {
            m_read = wrapIndex(m_read + 1);
            len++;
        }
        *pLen = len;
        *pFlags = flags;

        return true;
    }

    bool isStart(uint8_t flags)
    {
        return flags & Start;
    }

    bool isCommand(uint8_t flags)
    {
        return flags & Command;
    }

    bool isEnd(uint8_t flags)
    {
        return flags & End;
    }

protected:
    void writeByteAndFlag(uint8_t byte, uint8_t flags)
    {
        while (isFull()) {
            // Busy wait.
        }
        m_bytes[m_write] = byte;
        m_flags[m_write] = flags;
        m_write = wrapIndex(m_write + 1);
        m_state = None;
    }

    uint32_t wrapIndex(uint32_t index)
    {
        return index & (BUFFER_SIZE - 1);
    }

    enum State {
        None = 0,
        Start = (1 << 0),
        Command = (1 << 1),
        End = (1 << 2)
    };

    volatile uint32_t   m_write;
    volatile uint32_t   m_read;
    State               m_state;
    uint8_t             m_bytes[BUFFER_SIZE];
    uint8_t             m_flags[BUFFER_SIZE];
};



// CLASS DEFINITION --------------------------------------------------------

/*!
  @brief  Adafruit_SPITFT is an intermediary class between Adafruit_GFX
          and various hardware-specific subclasses for different displays.
          It handles certain operations that are common to a range of
          displays (address window, area fills, etc.). Originally these were
          all color TFT displays interfaced via SPI, but it's since expanded
          to include color OLEDs and parallel-interfaced TFTs. THE NAME HAS
          BEEN KEPT TO AVOID BREAKING A LOT OF SUBCLASSES AND EXAMPLE CODE.
          Many of the class member functions similarly live on with names
          that don't necessarily accurately describe what they're doing,
          again to avoid breaking a lot of other code. If in doubt, read
          the comments.
*/
class Adafruit_SPITFT : public Adafruit_GFX {

public:
  // CONSTRUCTORS --------------------------------------------------------

  // Hardware SPI constructor using an arbitrary SPI peripheral: expects
  // width & height (rotation 0), SPI pointer, 4 signal pins (mosi, sck,
  // cs, dc) and optional reset pin.
  Adafruit_SPITFT(uint16_t width, uint16_t height, nrf_drv_spi_t* pSpi,
                  uint8_t mosiPin, uint8_t sckPin, uint8_t csPin, uint8_t dcPin,
                  uint8_t rstPin = NRF_DRV_SPI_PIN_NOT_USED);

  // CLASS MEMBER FUNCTIONS ----------------------------------------------

  // These first two functions MUST be declared by subclasses:

  /*!
      @brief  Display-specific initialization function.
      @param  frequency  SPI frequency - one of NRF_DRV_SPI_FREQ_*.
  */
  virtual void begin(nrf_drv_spi_frequency_t frequency) = 0;

  /*!
      @brief  Set up the specific display hardware's "address window"
              for subsequent pixel-pushing operations.
      @param  x  Leftmost pixel of area to be drawn (MUST be within
                 display bounds at current rotation setting).
      @param  y  Topmost pixel of area to be drawn (MUST be within
                 display bounds at current rotation setting).
      @param  w  Width of area to be drawn, in pixels (MUST be >0 and,
                 added to x, within display bounds at current rotation).
      @param  h  Height of area to be drawn, in pixels (MUST be >0 and,
                 added to x, within display bounds at current rotation).
  */
  virtual void setAddrWindow(uint16_t x, uint16_t y, uint16_t w,
                             uint16_t h) = 0;

  // Remaining functions do not need to be declared in subclasses
  // unless they wish to provide hardware-specific optimizations.
  // Brief comments here...documented more thoroughly in .cpp file.

  // Subclass' begin() function invokes this to initialize hardware.
  // frequency must be one of NRF_DRV_SPI_FREQ_* values. spiMode must
  // be one of the NRF_DRV_SPI_MODE_n values.
  void initSPI(nrf_drv_spi_frequency_t frequency = NRF_DRV_SPI_FREQ_8M, nrf_drv_spi_mode_t mode = NRF_DRV_SPI_MODE_0);
  // Chip select and/or hardware SPI transaction start as needed:
  void startWrite(void);
  // Chip deselect and/or hardware SPI transaction end as needed:
  void endWrite(void);
  void sendCommand(uint8_t commandByte, const uint8_t *dataBytes = NULL, uint8_t numDataBytes = 0);

  // These functions require a chip-select and/or SPI transaction
  // around them. Higher-level graphics primitives might start a
  // single transaction and then make multiple calls to these functions
  // (e.g. circle or text rendering might make repeated lines or rects)
  // before ending the transaction. It's more efficient than starting a
  // transaction every time.
  void writePixel(int16_t x, int16_t y, uint16_t color);
  void writePixels(const uint16_t *colors, uint32_t len, bool block = true, bool bigEndian = false);
  void writeColor(uint16_t color, uint32_t len);
  void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  // This is a new function, similar to writeFillRect() except that
  // all arguments MUST be onscreen, sorted and clipped. If higher-level
  // primitives can handle their own sorting/clipping, it avoids repeating
  // such operations in the low-level code, making it potentially faster.
  // CALLING THIS WITH UNCLIPPED OR NEGATIVE VALUES COULD BE DISASTROUS.
  inline void writeFillRectPreclipped(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

  // These functions are similar to the 'write' functions above, but with
  // a chip-select and/or SPI transaction built-in. They're typically used
  // solo -- that is, as graphics primitives in themselves, not invoked by
  // higher-level primitives (which should use the functions above).
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);

  using Adafruit_GFX::drawRGBBitmap; // Check base class first
  void drawRGBBitmap(int16_t x, int16_t y, uint16_t *pcolors, int16_t w, int16_t h);

  void invertDisplay(bool i);
  uint16_t color565(uint8_t r, uint8_t g, uint8_t b);

  void spiWrite(uint8_t b);          // Write single byte as DATA
  void spiWrite16(uint16_t h);       // Write 16-bit value as DATA
  void spiWrite32(uint32_t h);       // Write 32-bit value as DATA
  void writeCommand(uint8_t cmd);    // Write single byte as COMMAND

protected:
  // CLASS INSTANCE VARIABLES --------------------------------------------
    nrf_drv_spi_t*          m_pSpi;
    uint8_t                 m_mosiPin;
    uint8_t                 m_sckPin;
    uint8_t                 m_rstPin;             ///< Reset pin # (or -1)
    uint8_t                 m_csPin;              ///< Chip select pin # (or -1)
    uint8_t                 m_dcPin;              ///< Data/command pin #

    int16_t                 _xstart = 0;          ///< Internal framebuffer X offset
    int16_t                 _ystart = 0;          ///< Internal framebuffer Y offset
    uint8_t                 invertOnCommand = 0;  ///< Command to enable invert mode
    uint8_t                 invertOffCommand = 0; ///< Command to disable invert mode

    CircularBuffer<256>     m_buffer;
    volatile uint32_t       m_startCount;
    volatile uint32_t       m_finCount;

    static Adafruit_SPITFT* g_pSingleton;

    static void _spiHandler(nrf_drv_spi_evt_t const * pEvent)
    {
        g_pSingleton->spiHandler(pEvent);
    }
    void spiHandler(nrf_drv_spi_evt_t const * pEvent);

    void startTransmission()
    {
        if (m_finCount == m_startCount) {
            // All SPI transmits have been completed so call spiHandler manually to kick-off a new block of
            // transmits.
            spiHandler(NULL);
        }
    }
};

#endif // end _ADAFRUIT_SPITFT_H_
