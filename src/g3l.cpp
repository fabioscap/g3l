#include "g3l.h"

namespace k_api = Kinova::Api;

g3l::G3L::G3L() {}

void g3l::G3L::connect() {
  if (connected_) {
    //
    return;
  }

  using namespace k_api;

  auto create_session_info = k_api::Session::CreateSessionInfo();
  create_session_info.set_username(uname_);
  create_session_info.set_password(pword_);
  create_session_info.set_session_inactivity_timeout(60000);
  create_session_info.set_connection_inactivity_timeout(2000);

  high_level_session_ = std::make_unique<Session_<TransportClientTcp>>(
      create_session_info, ipaddr_, high_level_port_);
  low_level_session_ = std::make_unique<Session_<TransportClientUdp>>(
      create_session_info, ipaddr_, low_level_port_);

  connected_ = true;
}

void g3l::G3L::disconnect() {
  if (!connected_) {
    //
    return;
  }

  high_level_session_.reset();
  low_level_session_.reset();

  connected_ = false;
}

tinyxml2::XMLDocument* g3l::G3L::parse_urdf(const std::string& xml_path) {
  auto doc = new tinyxml2::XMLDocument;

  doc->LoadFile(xml_path.c_str());

  return doc;
}

template struct g3l::Session_<Kinova::Api::TransportClientTcp>;
template struct g3l::Session_<Kinova::Api::TransportClientUdp>;