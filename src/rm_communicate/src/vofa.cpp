#include "rm_communicate/vofa.hpp"

#include <boost/asio.hpp>

namespace rm_communicate
{
class Vofa::Impl
{
 private:
  boost::asio::io_context io_context_;
  boost::asio::ip::udp::socket udp_socket_;
  boost::asio::ip::udp::endpoint vofa_endpoint_;

 public:
  Impl(std::string vofa_ip, uint16_t vofa_port) : io_context_{}, udp_socket_{io_context_}
  {
    using namespace boost::asio;
    udp_socket_.open(ip::udp::v4());

    vofa_endpoint(vofa_ip, vofa_port);
  }

  void vofa_endpoint(std::string vofa_ip, uint16_t vofa_port)
  {
    using namespace boost::asio;
    vofa_endpoint_ = ip::udp::endpoint(ip::address::from_string(vofa_ip), vofa_port);
  }

  void rawdata(const std::vector<uint8_t> &data)
  {
    using namespace boost::asio;
    udp_socket_.send_to(buffer(data), vofa_endpoint_);
  }
};

Vofa::Vofa(std::string vofa_ip, uint16_t vofa_port)
    : pimpl{std::make_unique<Impl>(vofa_ip, vofa_port)}
{
}
Vofa::~Vofa() = default;

void Vofa::vofa_endpoint(std::string vofa_ip, uint16_t vofa_port)
{
  pimpl->vofa_endpoint(vofa_ip, vofa_port);
}

void Vofa::rawdata(const std::vector<uint8_t> &data) { pimpl->rawdata(data); }
}  // namespace rm_communicate