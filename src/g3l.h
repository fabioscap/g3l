#include <functional>

#include "KError.h"
#include "RouterClient.h"
#include "SessionManager.h"
#include "TransportClientTcp.h"
#include "TransportClientUdp.h"
#include "tinyxml2.h"

namespace g3l {

template <typename TransportType_>
class Session_;
/*
Connect and disconnect to the robot
*/
class G3L {
 public:
  G3L();
  void connect();
  void disconnect();

  tinyxml2::XMLDocument* parse_urdf(const std::string&);

  // private:
  bool connected_{false};

  const std::string ipaddr_ = "192.168.1.10";
  const std::string uname_ = "admin";
  const std::string pword_ = "admin";
  const size_t high_level_port_ = 10000;
  const size_t low_level_port_ = 10001;

  std::unique_ptr<Session_<Kinova::Api::TransportClientTcp>>
      high_level_session_;
  std::unique_ptr<Session_<Kinova::Api::TransportClientUdp>> low_level_session_;
};

template <typename TransportType_>
struct Session_ {
  Session_(Kinova::Api::Session::CreateSessionInfo info, std::string ipaddr,
           size_t port) {
    transport_ = std::make_unique<TransportType_>();
    router_ = std::make_unique<Kinova::Api::RouterClient>(transport_.get(),
                                                          error_callback_);
    transport_->connect(ipaddr, port);

    session_manager_ =
        std::make_unique<Kinova::Api::SessionManager>(router_.get());
    session_manager_->CreateSession(info);
  }
  ~Session_() {
    session_manager_->CloseSession();
    router_->SetActivationStatus(false);
    transport_->disconnect();

    session_manager_.reset();
    router_.reset();
    transport_.reset();
  }
  std::unique_ptr<TransportType_> transport_;
  std::unique_ptr<Kinova::Api::RouterClient> router_;
  std::unique_ptr<Kinova::Api::SessionManager> session_manager_;

  std::function<void(Kinova::Api::KError)> error_callback_ =
      [](Kinova::Api::KError err) {
        std::cout << "_________ callback error _________" << err.toString()
                  << std::endl;
      };
};

}  // namespace g3l