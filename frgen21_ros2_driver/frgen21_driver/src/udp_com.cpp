#include "udp_com.h"

UdpReceiver::UdpReceiver(int local_port, const std::string &local_address,
                         const std::string &multicast_address,
                         std::size_t buffer_size)
    : io_context_(), socket_(io_context_), buffer_(buffer_size),
      is_receiving_(false) {
  try {
    udp::endpoint local_endpoint = udp::endpoint(udp::v4(), local_port);

    boost::asio::socket_base::reuse_address option(true);
    socket_.open(local_endpoint.protocol());
    socket_.set_option(option);
    socket_.bind(local_endpoint);

    if (!multicast_address.empty()) {
      socket_.set_option(boost::asio::ip::multicast::join_group(
          boost::asio::ip::address::from_string(multicast_address).to_v4(),
          boost::asio::ip::address::from_string(local_address).to_v4()));
    }
  } catch (const boost::system::system_error &e) {
    std::cerr << "Error initializing UdpReceiver: " << e.what() << std::endl;
    throw;
  }
}

UdpReceiver::~UdpReceiver() { stopReceive(); }

void UdpReceiver::setBufferSize(std::size_t new_buffer_size) {
  buffer_.resize(new_buffer_size);
}

std::vector<char> UdpReceiver::receive(const std::string &filter_ip) {
  udp::endpoint sender_endpoint;
  try {
    std::size_t bytes_received =
        socket_.receive_from(boost::asio::buffer(buffer_), sender_endpoint);

    if (filter_ip.empty() ||
        sender_endpoint.address().to_string() == filter_ip) {
      return std::vector<char>(buffer_.begin(),
                               buffer_.begin() + bytes_received);
    }
  } catch (const boost::system::system_error &e) {
    std::cerr << "Error receiving data: " << e.what() << std::endl;
    return std::vector<char>(); // return empty vector on error
  }

  return std::vector<char>(); // filter did not match, return empty vector
}

void UdpReceiver::startAsyncReceive(
    std::function<void(const std::vector<char> &)> callback,
    const std::string &filter_ip) {
  try {
    if (!is_receiving_.load()) {
      is_receiving_.store(true);

      // Start the async receive
      socket_.async_receive_from(
          boost::asio::buffer(buffer_), sender_endpoint_,
          [this, filter_ip, callback](const boost::system::error_code &error,
                                      std::size_t bytes_received) {
            handleAsyncReceive(error, bytes_received, filter_ip, callback);
          });

      if (!io_thread_.joinable()) {
        io_thread_ = std::thread([this]() { io_context_.run(); });
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Error starting async receive: " << e.what() << std::endl;
    throw;
  }
}

void UdpReceiver::handleAsyncReceive(
    const boost::system::error_code &error, std::size_t bytes_received,
    const std::string &filter_ip,
    std::function<void(const std::vector<char> &)> callback) {
  if (!error) {
    if (filter_ip.empty() ||
        sender_endpoint_.address().to_string() == filter_ip) {
      callback(
          std::vector<char>(buffer_.begin(), buffer_.begin() + bytes_received));
    } else {
      callback(
          std::vector<char>()); // Return empty vector if filter did not match
    }

    // Reset the receiving flag to allow continuation
    is_receiving_.store(false);

    // Continue receiving data by restarting the async receive
    startAsyncReceive(callback, filter_ip);
  } else {
    std::cerr << "Receive error: " << error.message() << std::endl;
    is_receiving_.store(false); // Stop receiving on error
  }
}

void UdpReceiver::stopReceive() {
  try {
    if (is_receiving_.load()) {
      socket_.cancel();   // Cancel any outstanding async operations
      io_context_.stop(); // Stop the io_context

      if (io_thread_.joinable()) {
        io_thread_.join(); // Join the thread
      }

      is_receiving_.store(false);
    }
  } catch (const boost::system::system_error &e) {
    std::cerr << "Error stopping receive: " << e.what() << std::endl;
  }
}

UDPSender::UDPSender(const std::string &remoteIp, int remotePort, int localPort)
    : io_context_(), socket_(io_context_),
      remote_endpoint_(boost::asio::ip::address::from_string(remoteIp),
                       remotePort),
      local_port_(localPort) {
  try {
    socket_.open(udp::v4());
    socket_.set_option(boost::asio::socket_base::reuse_address(true));
    socket_.bind(udp::endpoint(udp::v4(), localPort));
  } catch (const boost::system::system_error &e) {
    std::cerr << "Error initializing UDPSender: " << e.what() << std::endl;
    throw; // rethrow exception after logging the error
  }
}

void UDPSender::send(const std::vector<char> &message) {
    try {
        socket_.send_to(boost::asio::buffer(message), remote_endpoint_);
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error sending data: " << e.what() << std::endl;
    }
}