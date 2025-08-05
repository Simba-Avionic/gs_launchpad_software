
#include "GSUART.hpp"

#include <stdint.h>
#include <errno.h>
#include <string.h>
#if GSUART_PLATFORM == GSUART_PLATFORM_RPI_UBUNTU
#include <unistd.h>
#include <termio.h>
#include <fcntl.h>
#endif

#if GSUART_PLATFORM == GSUART_PLATFORM_ARDUINO
#include "wiring.c"  // millis();
#endif

namespace GSUART {
Message::Message(MsgID id)
  : id(id) {}

MsgID Message::getID() const {
  return id;
}

void UARTStatistics::calculatePerSecValues() {
  auto now_millis = getCurrentTimeMillis();
  auto duration = static_cast<float>(now_millis - lastCalcTime) / 1000.0f;

  if (duration == 0.0f) {
    return;
  }

  stats.messagesSentPerSec = static_cast<unsigned int>((stats.totalMessagesSent - lastSecTotalMessagesSent) / duration);
  stats.messagesRecPerSec = static_cast<unsigned int>((stats.totalMessagesReceived - lastSecTotalMessagesReceived) / duration);
  stats.goodMessagesReceivedPerSec = static_cast<unsigned int>((stats.goodMessagesReceived - lastSecGoodMessagesReceived) / duration);
  stats.bytesRecPerSec = static_cast<unsigned int>((stats.totalBytesReceived - lastSecTotalBytesReceived) / duration);
  stats.bytesSentPerSec = static_cast<unsigned int>((stats.totalBytesSent - lastSecTotalBytesSent) / duration);

  stats.goodMessagesReceivedPerSecRatio = (stats.messagesRecPerSec > 0) ? static_cast<float>(stats.goodMessagesReceivedPerSec) / stats.messagesRecPerSec : 0.0f;

  lastSecTotalMessagesSent = stats.totalMessagesSent;
  lastSecTotalMessagesReceived = stats.totalMessagesReceived;
  lastSecGoodMessagesReceived = stats.goodMessagesReceived;
  lastSecTotalBytesReceived = stats.totalBytesReceived;
  lastSecTotalBytesSent = stats.totalBytesSent;
  lastCalcTime = now_millis;
}

// arduino is different here
#if GSUART_PLATFORM == GSUART_PLATFORM_RPI_UBUNTU
Messenger::Messenger(std::string uart_device)
  : serialPort(uart_device) {
  openSerialPort();
}
#endif
#if GSUART_PLATFORM == GSUART_PLATFORM_ARDUINO
Messenger::Messenger(HardwareSerial* serialPort)
  : serialPort(serialPort)
{
  openSerialPort();
}
#endif

Messenger::~Messenger() {
  if (receivedMsg) {
    delete receivedMsg;
    receivedMsg = nullptr;
  }
}

void Messenger::send(const Message& msg) {
  Byte frame[WRITE_BUFF_SIZE];
  frame[0] = RAMKA_START;
  frame[1] = (Byte)msg.getID();

  size_t idx = 2;
  Byte msgSerializedBytes[WRITE_BUFF_SIZE];
  size_t msgSerializedSize;
  msg.serialize(msgSerializedBytes, &msgSerializedSize);
  for (size_t i = 0; i < msgSerializedSize; i++)
    putByteIntoFrame(msgSerializedBytes[i], frame, idx);

  Byte checksum = 0;
  for (size_t i = 1; i < idx; i++)
    checksum += frame[i];
  putByteIntoFrame(checksum, frame, idx);
  frame[idx++] = RAMKA_STOP;

  writeToSerialPort(frame, idx);
  uartStats.stats.totalMessagesSent++;
  uartStats.stats.totalBytesSent += idx;
}

const Message* Messenger::receive() {
  #if GSUART_PLATFORM == GSUART_PLATFORM_ARDUINO
    if (serialPort->available() <= 0)
      return nullptr;
  #endif

  if (receivedMsg) {
    delete receivedMsg;
    receivedMsg = nullptr;
  }

  // gdyby brakowalo RAMU na arduino to ten buffor mozna zmniejszyc
  Byte read_buff[READ_BUFF_SIZE*2];
  if (extra_buff_data_size > 0) // jesli cos zostalo z poprzedniego odczytu
    memcpy(read_buff, extra_buff, extra_buff_data_size);
  int n = readFromSerialPort(read_buff+extra_buff_data_size, READ_BUFF_SIZE);
  extra_buff_data_size = 0;

  uartStats.stats.totalBytesReceived += n;
  for (int i = 0; i < n; i++) {
    Byte b = read_buff[i];
    receive_buff[receive_buff_idx++] = b;

    if (receive_buff_idx >= RECEIVE_BUFF_SIZE)  // zabezpieczenie przed overflowem
    {
      uartStats.stats.bufforOverflows++;
      receive_buff_idx = 0;
      continue;
    }

    if (b == RAMKA_START) {
      receive_buff_idx = 0;
      receive_buff[receive_buff_idx++] = b;
    } else if (b == RAMKA_STOP) {
      decodeMsg(receive_buff, receive_buff_idx);
      receive_buff_idx = 0;

      extra_buff_data_size = n-i-1;
      memcpy(extra_buff, read_buff+i+1, extra_buff_data_size);
      break;
    }
  }
  return receivedMsg;
}

void Messenger::sendUartStats() {
    uartStats.calculatePerSecValues();
    MsgUartStats msg;
    msg.stats = uartStats.stats;
    send(msg);
}

const UARTStatistics& Messenger::getStats() {
  uartStats.calculatePerSecValues();
  return uartStats;
}

void Messenger::decodeMsg(Byte* msg_bytes, const size_t size) {
  if (size < 3)
    return;
  if (msg_bytes[0] != RAMKA_START || msg_bytes[size - 1] != RAMKA_STOP)
    return;

  uartStats.stats.totalMessagesReceived++;
  // remove special values, in place algorithm :O <- zamiast przepisywac do nowego bufora mozna przenosic w tym samym buforze
  size_t conv_idx = 0;
  for (size_t i = 1; i < size - 1; i++)  // bez bajt start i stop
  {
    Byte b = msg_bytes[i];
    if (b == RAMKA_SPECIAL) {
      i++;
      msg_bytes[i] += R_SPECIAL_DIF;
      msg_bytes[conv_idx++] = msg_bytes[i];
    } else
      msg_bytes[conv_idx++] = b;
  }

  // sprawdzenie checksumy
  Byte checksum = 0;
  for (size_t i = 0; i < conv_idx - 1; i++)  // exclude last byte (checksum)
  {
    checksum += msg_bytes[i];
  }

  if (checksum != msg_bytes[conv_idx - 1]) {
    return;
  }
  uartStats.stats.goodMessagesReceived++;

  if (receivedMsg) {
    uartStats.stats.messagesOverwritten++;
    delete receivedMsg;
    receivedMsg = nullptr;
  }

  MsgID msgID = (MsgID)msg_bytes[0];
  switch (msgID) {
    case MsgID::TENSO:
      receivedMsg = new MsgTenso();
      break;
    case MsgID::TEMPERATURE:
      receivedMsg = new MsgTemperature();
      break;
    case MsgID::ZAWORY_STEROWANIE:
      receivedMsg = new MsgZaworySterowanie();
      break;
    case MsgID::ZAWORY_POZYCJA:
      receivedMsg = new MsgZaworyPozycja();
      break;
    case MsgID::PRESSURE:
      receivedMsg = new MsgPressure();
      break;
    case MsgID::UART_STATS:
      receivedMsg = new MsgUartStats();
      break;
    default:
      receivedMsg = nullptr;
      return;
  }
  receivedMsg->deserialize(msg_bytes + 1, conv_idx - 2);
}

// arduino is different here
void Messenger::openSerialPort() {
    #if GSUART_PLATFORM == GSUART_PLATFORM_RPI_UBUNTU
        serialPortFD = open(serialPort.c_str(), O_RDWR);
        struct termios tty;
        if (tcgetattr(serialPortFD, &tty) != 0) {
          printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
          exit(1);
        }

        tty.c_cflag &= ~PARENB;         // Clear parity bit, disabling parity (most common)
        tty.c_cflag &= ~CSTOPB;         // Clear stop field, only one stop bit used in communication (most common)
        tty.c_cflag &= ~CSIZE;          // Clear all bits that set the data size
        tty.c_cflag |= CS8;             // 8 bits per byte (most common)
        tty.c_cflag &= ~CRTSCTS;        // Disable RTS/CTS hardware flow control (most common)
        tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;                                                         // Disable echo
        tty.c_lflag &= ~ECHOE;                                                        // Disable erasure
        tty.c_lflag &= ~ECHONL;                                                       // Disable new-line echo
        tty.c_lflag &= ~ISIG;                                                         // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                       // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);  // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

        tty.c_cc[VTIME] = 10;  // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        // Set in/out baud rate to be 9600
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        // Save tty settings, also checking for error
        if (tcsetattr(serialPortFD, TCSANOW, &tty) != 0) {
          printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
          exit(1);
        }
        printf("Serial port opened: %s\n", serialPort.c_str());
    #endif
    #if GSUART_PLATFORM == GSUART_PLATFORM_ARDUINO
        // serialPort->begin(9600);  // does not work here, has to be done in setup()
    #endif
}

// arduino is different here
void Messenger::writeToSerialPort(const Byte* bytes, const size_t size) {
#if GSUART_PLATFORM == GSUART_PLATFORM_RPI_UBUNTU
  ssize_t res = write(serialPortFD, bytes, size);
  if (res != static_cast<ssize_t>(size)) {
    printf("Error writing to serial port: %s\n", strerror(errno));
  } else {
    printf("Successfully wrote %ld bytes to serial port.\n", res);
  }
#endif

#if GSUART_PLATFORM == GSUART_PLATFORM_ARDUINO
  int res = serialPort->write(bytes, size);
  if (res != static_cast<int>(size)) {
    printf("Error writing to serial port: %d\n", errno);
  } else {
    printf("Successfully wrote %ld bytes to serial port.\n", res);
  }
#endif
}

// arduino is different here
int Messenger::readFromSerialPort(Byte* bytes, const size_t size) {
#if GSUART_PLATFORM == GSUART_PLATFORM_RPI_UBUNTU
  int bytesRead = read(serialPortFD, bytes, size);
  if (bytesRead < 0) {
    printf("Error %i from read: %s\n", errno, strerror(errno));
  }
  return bytesRead;
#endif

#if GSUART_PLATFORM == GSUART_PLATFORM_ARDUINO
  int bytesRead = serialPort->readBytes(bytes, size);
  return bytesRead;
#endif
}


void MsgTenso::serialize(Byte* bytes_out, size_t* size_out) const {
  if (bytes_out == nullptr || size_out == nullptr) return;
  *size_out = sizeof(int32_t) * 2;
  memcpy(bytes_out, &tenso_left_raw, sizeof(int32_t));
  memcpy(bytes_out + sizeof(int32_t), &tenso_right_raw, sizeof(int32_t));
}

void MsgTenso::deserialize(const Byte* bytes_in, const size_t size_in) {
  if (bytes_in == nullptr || size_in < sizeof(int32_t) * 2) return;
  memcpy(&tenso_left_raw, bytes_in, size_in);
  memcpy(&tenso_right_raw, bytes_in + sizeof(int32_t), size_in);
}

void MsgTemperature::serialize(Byte* bytes_out, size_t* size_out) const {
  if (bytes_out == nullptr || size_out == nullptr) return;
  *size_out = sizeof(float);
  memcpy(bytes_out, &temperature_celsius, sizeof(float));
}

void MsgTemperature::deserialize(const Byte* bytes_in, const size_t size_in) {
  if (bytes_in == nullptr || size_in < sizeof(float)) return;
  memcpy(&temperature_celsius, bytes_in, size_in);
}

void MsgZaworySterowanie::serialize(Byte* bytes_out, size_t* size_out) const {
  if (bytes_out == nullptr || size_out == nullptr) return;
  *size_out = sizeof(uint8_t);
  uint8_t bools = 0;
  bools |= (valve_feed_oxidizer ? 1 : 0) << 0;
  bools |= (valve_feed_pressurizer ? 1 : 0) << 1;
  bools |= (valve_vent_oxidizer ? 1 : 0) << 2;
  bools |= (valve_vent_pressurizer ? 1 : 0) << 3;
  bools |= (decoupler_oxidizer ? 1 : 0) << 4;
  bools |= (decoupler_pressurizer ? 1 : 0) << 5;
  memcpy(bytes_out, &bools, sizeof(uint8_t));
}

void MsgZaworySterowanie::deserialize(const Byte* bytes_in, const size_t size_in) {
  if (bytes_in == nullptr || size_in < sizeof(uint8_t)) return;
  uint8_t bools;
  memcpy(&bools, bytes_in, sizeof(uint8_t));
  valve_feed_oxidizer = (bools & (1 << 0)) != 0;
  valve_feed_pressurizer = (bools & (1 << 1)) != 0;
  valve_vent_oxidizer = (bools & (1 << 2)) != 0;
  valve_vent_pressurizer = (bools & (1 << 3)) != 0;
  decoupler_oxidizer = (bools & (1 << 4)) != 0;
  decoupler_pressurizer = (bools & (1 << 5)) != 0;
}

void MsgZaworyPozycja::serialize(Byte* bytes_out, size_t* size_out) const {
  if (bytes_out == nullptr || size_out == nullptr) return;
  uint8_t bools = 0;
  bools |= (valve_vent_oxidizer ? 1 : 0) << 0;
  bools |= (valve_vent_pressurizer ? 1 : 0) << 1;
  bools |= (decoupler_oxidizer ? 1 : 0) << 2;
  bools |= (decoupler_pressurizer ? 1 : 0) << 3;
  memcpy(bytes_out, &valve_feed_oxidizer, sizeof(int8_t));
  memcpy(bytes_out + sizeof(int8_t), &valve_feed_pressurizer, sizeof(int8_t));
  memcpy(bytes_out + sizeof(int8_t) * 2, &bools, sizeof(uint8_t));
  *size_out = sizeof(int8_t) * 2 + sizeof(uint8_t);
}

void MsgZaworyPozycja::deserialize(const Byte* bytes_in, const size_t size_in) {
  if (bytes_in == nullptr || size_in < sizeof(int8_t) * 2 + sizeof(uint8_t)) return;
  memcpy(&valve_feed_oxidizer, bytes_in, sizeof(int8_t));
  memcpy(&valve_feed_pressurizer, bytes_in + sizeof(int8_t), sizeof(int8_t));
  uint8_t bools;
  memcpy(&bools, bytes_in + sizeof(int8_t) * 2, sizeof(uint8_t));
  valve_vent_oxidizer = (bools & (1 << 0)) != 0;
  valve_vent_pressurizer = (bools & (1 << 1)) != 0;
  decoupler_oxidizer = (bools & (1 << 2)) != 0;
  decoupler_pressurizer = (bools & (1 << 3)) != 0;  
}

void MsgPressure::serialize(Byte* bytes_out, size_t* size_out) const {
  if (bytes_out == nullptr || size_out == nullptr) return;
  *size_out = sizeof(float);
  memcpy(bytes_out, &pressure_bar, sizeof(float));
}

void MsgPressure::deserialize(const Byte* bytes_in, const size_t size_in) {
  if (bytes_in == nullptr || size_in < sizeof(float)) return;
  memcpy(&pressure_bar, bytes_in, size_in);
}

void MsgUartStats::serialize(Byte* bytes_out, size_t* size_out) const {
  if (bytes_out == nullptr || size_out == nullptr) return;
  *size_out = UARTStatistics::Stats::_STRUCT_SIZE;
  memcpy(bytes_out, &stats, UARTStatistics::Stats::_STRUCT_SIZE);
}

void MsgUartStats::deserialize(const Byte* bytes_in, const size_t size_in) {
  if (bytes_in == nullptr || size_in < UARTStatistics::Stats::_STRUCT_SIZE) return;
  memcpy(&stats, bytes_in, size_in);
}

void MsgPowerTanking::serialize(Byte* bytes_out, size_t* size_out) const {
  if (bytes_out == nullptr || size_out == nullptr) return;
  #define SIZE MsgPowerTanking::PowerSensor::_STRUCT_SIZE
  *size_out = SIZE * 2;
  memcpy(bytes_out, &v7_4, SIZE);
  memcpy(bytes_out + SIZE, &v12, SIZE);
}

void MsgPowerTanking::deserialize(const Byte* bytes_in, const size_t size_in) {
  #define SIZE MsgPowerTanking::PowerSensor::_STRUCT_SIZE
  if (bytes_in == nullptr || size_in < SIZE * 2) return;
  memcpy(&v7_4, bytes_in, SIZE);
  memcpy(&v12, bytes_in + SIZE, SIZE);
}

void putByteIntoFrame(Byte byte, Byte* bytes, size_t& idx) {
  if (byte == RAMKA_STOP || byte == RAMKA_START || byte == RAMKA_SPECIAL) {
    bytes[idx++] = RAMKA_SPECIAL;
    bytes[idx++] = byte - R_SPECIAL_DIF;
  } else bytes[idx++] = byte;
}

// arduino is different here
uint64_t getCurrentTimeMillis() {
    #if GSUART_PLATFORM == GSUART_PLATFORM_RPI_UBUNTU
      return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    #endif
    #if GSUART_PLATFORM == GSUART_PLATFORM_ARDUINO
      return millis();
    #endif
}
}