#include "g3l.h"

int main() {
  namespace k_api = Kinova::Api;

  g3l::G3L client;

  auto urdf =
      client.parse_urdf("/home/fabioscap/Desktop/g3l/resources/gen3_lite.urdf");
  urdf->Print();
}