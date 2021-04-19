
#pragma once

#include <iostream>
#include <string>
namespace isaac
{
  namespace pololu
  {

    class MaestroConfig
    {

    public:
      enum Type : int
      {
        MICRO_6 = 0,
        MINI_12 = 1,
        MINI_18 = 2,
        MINI_24 = 3
      };
      enum Protocol : int
      {
        COMPACT,
        POLOLU,
        MINI_SSC
      };
      MaestroConfig(Type type, Protocol protocol, std::string device_file, unsigned char device_number)
          : type_(type), protocol_(protocol), device_file_(device_file), device_number_(device_number){};

      MaestroConfig()
          : type_(Type::MINI_12), protocol_(Protocol::COMPACT), device_file_("/dev/ttyACM0"), device_number_(0x0C){};

      virtual ~MaestroConfig(){};

      Type getType() { return type_; }
      Protocol getProtocol() { return protocol_; }
      std::string getDeviceFile() { return device_file_; }
      unsigned char getDeviceNumber() { return device_number_; }

    private:
      Type type_;
      Protocol protocol_;
      std::string device_file_;
      unsigned char device_number_;
    };

  }
}

