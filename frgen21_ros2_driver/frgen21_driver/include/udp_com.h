#pragma once
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using boost::asio::ip::udp;

class UdpReceiver {
public:
  UdpReceiver(int local_port, const std::string &local_address = "0.0.0.0",
              const std::string &multicast_address = "",
              std::size_t buffer_size = 65535);
  ~UdpReceiver();
  std::vector<char> receive(const std::string &filter_ip = "");
  void
  startAsyncReceive(std::function<void(const std::vector<char> &)> callback,
                    const std::string &filter_ip = "");
  void stopReceive();
  void setBufferSize(std::size_t new_buffer_size);

private:
  void
  handleAsyncReceive(const boost::system::error_code &error,
                     std::size_t bytes_received, const std::string &filter_ip,
                     std::function<void(const std::vector<char> &)> callback);
  boost::asio::io_context io_context_;
  udp::socket socket_;
  std::vector<char> buffer_;
  udp::endpoint sender_endpoint_;
  std::thread io_thread_;
  std::atomic<bool> is_receiving_; // Use atomic to ensure thread safety
};

class UDPSender {
public:
  UDPSender(const std::string &remoteIp, int remotePort, int localPort = 0);
  void send(const std::vector<char> &message);

private:
  boost::asio::io_context io_context_;
  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  int local_port_;
};
