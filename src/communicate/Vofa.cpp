#include "Vofa.hpp"

namespace rmcv::communicate
{

    Vofa::Vofa(std::string vofa_ip, uint16_t vofa_port) : io_context_{}, udp_socket_{io_context_}
    {
        using namespace boost::asio;
        udp_socket_.open(ip::udp::v4());

        vofa_endpoint(vofa_ip, vofa_port);
    }

    void Vofa::vofa_endpoint(std::string vofa_ip, uint16_t vofa_port)
    {
        using namespace boost::asio;
        vofa_endpoint_ = ip::udp::endpoint(ip::address::from_string(vofa_ip), vofa_port);
    }

    void Vofa::rawdata(std::vector<uint8_t> data)
    {
        using namespace boost::asio;
        udp_socket_.send_to(buffer(data), vofa_endpoint_);
    }
}