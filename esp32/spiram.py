# micropython ESP32
# SPI RAM test R/W

# AUTHOR=EMARD
# LICENSE=BSD

# this code is SPI master to FPGA SPI slave

from machine import SPI, Pin
from micropython import const
from time import sleep_ms

class spiram:
  def __init__(self):
    self.led = Pin(5, Pin.OUT)
    self.led.off()
    self.spi_channel = const(1)
    self.init_pinout_sd()
    self.spi_freq = const(2000000)
    self.hwspi=SPI(self.spi_channel, baudrate=self.spi_freq, polarity=0, phase=0, bits=8, firstbit=SPI.MSB, sck=Pin(self.gpio_sck), mosi=Pin(self.gpio_mosi), miso=Pin(self.gpio_miso))

  @micropython.viper
  def init_pinout_sd(self):
    self.gpio_sck  = const(16)
    self.gpio_mosi = const(4)
    self.gpio_miso = const(12)

  # read from file -> write to SPI RAM
  def load_stream(self, filedata, addr=0, blocksize=1024):
    block = bytearray(blocksize)
    self.led.on()
    self.hwspi.write(bytearray([0x00, (addr >> 24) & 0xFF, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF]))
    while True:
      if filedata.readinto(block):
        self.hwspi.write(block)
      else:
        break
    self.led.off()

  # read from SPI RAM -> write to file
  def save_stream(self, filedata, addr=0, length=1024, blocksize=1024):
    bytes_saved = 0
    block = bytearray(blocksize)
    self.led.on()
    self.hwspi.write(bytearray([0x01,(addr >> 8) & 0xFF, addr & 0xFF, 0x00]))
    while bytes_saved < length:
      self.hwspi.readinto(block)
      filedata.write(block)
      bytes_saved += len(block)
    self.led.off()

def load(filename, addr=0):
  s=spiram()
  s.led.on()
  s.hwspi.write(bytearray([0x00,0xFF,0xFF,0xFF,0xFF,0])) # not reset
  s.led.off()
  s.led.on()
  s.hwspi.write(bytearray([0x00,0xFF,0xFF,0xFF,0xFF,1])) # yes reset
  s.led.off()
  s.led.on()
  sleep_ms(100)
  s.led.on()
  s.hwspi.write(bytearray([0x00,0xFF,0xFF,0xFF,0xFF,0])) # not reset
  s.led.off()
  s.load_stream(open(filename, "rb"), addr)

def save(filename, addr=0, length=1024):
  s=spiram()
  f=open(filename, "wb")
  s.save_stream(f, addr, length)
  f.close()

def help():
  print("spiram.load(\"file.bin\",addr=0)")
  print("spiram.save(\"file.bin\",addr=0,length=1024)")

def test():
  s=spiram()
  
  # blink OSD off/on few times
  for i in range(6):
    s.led.on()
    s.hwspi.write(bytearray([0,0xFE,0x00,i&1])) # 1:show/0:hide OSD
    s.led.off()
    sleep_ms(200)

  # overwrite stars to the right of the screen (demonstrates writing single bytes)
  for i in range(24):
    s.led.on()
    s.hwspi.write(bytearray([0,i//(256//64),i*64+63,ord("*")])) # sets address and writes content
    s.led.off()
  
  # demonstrates writing multiple bytes, some text
  s.led.on()
  # two separate writes can be also joined in one bytearray but this way is
  # more readable as python code
  s.hwspi.write(bytearray([0,11,20])) # sets address
  s.hwspi.write(bytearray("ESP%d MICROPYTHON WAS HERE" % (32))) # writes content
  s.led.off()

# debug to manually write and read 4 bytes
#d=spiram.spiram()
#d.led.on(); d.hwspi.write(bytearray([0x00,0x00,0x00,0x40,0x41,0x42,0x43])); d.led.off()
#d.led.on(); d.hwspi.write(bytearray([0x01,0x00,0x00,0x00])); print(d.hwspi.read(4)); d.led.off()
