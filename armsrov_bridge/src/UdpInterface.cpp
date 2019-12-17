#include "inc/UdpInterface.h"

UdpServer::UdpServer(io_service& io_service, const unsigned short my_port) : _socket(io_service, udp::endpoint(udp::v4(), my_port))
{
  start_receive();
}

UdpServer::~UdpServer()
{
}

void UdpServer::start_receive()
{
  _socket.async_receive_from(buffer(_recv_buffer), _remote_endpoint,
                             boost::bind(&UdpServer::handle_receive, this, boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
}

void UdpServer::handle_receive(const boost::system::error_code& error, size_t bytes_transferred)
{
  if (!error || error == boost::asio::error::message_size)
  {
    // echo
    // std::string msg(_recv_buffer.data(), bytes_transferred);
    // std::cout << "Received message: " << msg << std::endl;
    // boost::shared_ptr<std::string> message(new std::string("Echo:" + msg));

    // _socket.async_send_to(boost::asio::buffer(*message), _remote_endpoint,
    //                       boost::bind(&UdpServer::handle_send, this, message, boost::asio::placeholders::error,
    //                                   boost::asio::placeholders::bytes_transferred));
    mavlinkHandler.parseChars(_recv_buffer.data(), bytes_transferred);

    start_receive();
  }
}

void UdpServer::handle_send(boost::shared_ptr<std::string> message, const boost::system::error_code& error,
                            std::size_t bytes_transferred)
{
  std::cout << "Send message: " << *message << "Len: " << bytes_transferred << std::endl;
}

UdpClient::UdpClient(string remote_ip, const unsigned short remote_port)
  : _socket(_io_service, udp::endpoint(udp::v4(), remote_port - 2))
  , _remote_endpoint(address_v4::from_string(remote_ip), remote_port)
{
  _socket.set_option(socket_base::broadcast(true));
}

UdpClient::~UdpClient()
{
}

void UdpClient::sendData(char* data, size_t len)
{
  _socket.send_to(buffer(data, len), _remote_endpoint);
}
