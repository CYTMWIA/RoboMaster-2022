// Linux 串口读写
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

#include "serial_port.hpp"

// Linux headers
#include <errno.h>  // Error integer and strerror() function
#include <fcntl.h>  // Contains file controls like O_RDWR
#include <sys/ioctl.h>
#include <termios.h>  // Contains POSIX terminal control definitions
#include <unistd.h>   // write(), read(), close()

// C++
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace
{
auto to_linux_baud_rate(uint32_t baud)
{
  switch (baud)
  {
    case 0:
      return B0;
    case 50:
      return B50;
    case 75:
      return B75;
    case 110:
      return B110;
    case 134:
      return B134;
    case 150:
      return B150;
    case 200:
      return B200;
    case 300:
      return B300;
    case 600:
      return B600;
    case 1200:
      return B1200;
    case 1800:
      return B1800;
    case 2400:
      return B2400;
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
  }
  throw std::invalid_argument("Not support custom baud rates.");
}

}  // namespace

namespace rm_serial
{
struct SerialPort::Impl
{
  int serial_fd_;
  struct termios tty_;

  std::string dev_;
  uint32_t baud_rate_;
};
SerialPort::~SerialPort() { close(pimpl_->serial_fd_); };
SerialPort::SerialPort() : pimpl_(new Impl()) {}

void SerialPort::open(const std::string& dev, uint32_t baud_rate)
{
  pimpl_->dev_ = dev;
  pimpl_->baud_rate_ = baud_rate;

  pimpl_->serial_fd_ = ::open(dev.c_str(), O_RDWR);

  if (pimpl_->serial_fd_ < 0) throw std::runtime_error(strerror(errno));

  if (tcgetattr(pimpl_->serial_fd_, &(pimpl_->tty_)) != 0)
    throw std::runtime_error(strerror(errno));

  // Control Modes (c_cflags)
  pimpl_->tty_.c_cflag &= ~PARENB;         // 无校验位
  pimpl_->tty_.c_cflag &= ~CSTOPB;         // 停止位：1
  pimpl_->tty_.c_cflag |= CS8;             // 数据位：8
  pimpl_->tty_.c_cflag &= ~CRTSCTS;        // 关闭流控
  pimpl_->tty_.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  // Local Modes (c_lflag)
  pimpl_->tty_.c_lflag &= ~ICANON;  // Disable canonical mode
  pimpl_->tty_.c_lflag &= ~ECHO;    // Disable echo
  pimpl_->tty_.c_lflag &= ~ECHOE;   // Disable erasure
  pimpl_->tty_.c_lflag &= ~ECHONL;  // Disable new-line echo
  pimpl_->tty_.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP

  // Input Modes (c_iflag)
  pimpl_->tty_.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
  pimpl_->tty_.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                            ICRNL);  // Disable any special handling of received bytes

  // Output Modes (c_oflag)
  pimpl_->tty_.c_oflag &=
      ~OPOST;  // Prevent special interpretation of output bytes (e.g. newline chars)
  pimpl_->tty_.c_oflag &= ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

  // VMIN and VTIME (c_cc)
  // VMIN = 0, VTIME = 0: No blocking, return immediately with what is available
  pimpl_->tty_.c_cc[VTIME] = 0;
  pimpl_->tty_.c_cc[VMIN] = 0;

  // Baud Rate
  cfsetispeed(&(pimpl_->tty_), to_linux_baud_rate(baud_rate));
  cfsetospeed(&(pimpl_->tty_), to_linux_baud_rate(baud_rate));

  if (tcsetattr(pimpl_->serial_fd_, TCSANOW, &(pimpl_->tty_)) != 0)
    throw std::runtime_error(strerror(errno));
}

void SerialPort::reopen()
{
  close(pimpl_->serial_fd_);
  open(pimpl_->dev_, pimpl_->baud_rate_);
}

uint32_t SerialPort::available()
{
  int bytes;
  int res = ioctl(pimpl_->serial_fd_, FIONREAD, &bytes);
  if (res < 0) throw std::runtime_error(strerror(errno));
  return (uint32_t)bytes;
}

void SerialPort::write(uint8_t* pdata, size_t size)
{
  int res = ::write(pimpl_->serial_fd_, pdata, size);
  if (res < 0) throw std::runtime_error(strerror(errno));
}

void SerialPort::read(uint8_t* pdata, size_t size)
{
  int res = ::read(pimpl_->serial_fd_, pdata, size);
  if (res < 0) throw std::runtime_error(strerror(errno));
}

}  // namespace rm_serial