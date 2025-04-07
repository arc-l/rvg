#include <Utils/Utils.h>
#include <Utils/Timer.h>
#include <Utils/Pragma.h>
#include <string>

using namespace RotationalVisibilityGraph;
using namespace Utils;

typedef double T;
int main(int argc, char *argv[]) {
  tinyxml2::XMLDocument pt;
  pt.LoadFile(argv[1]);
  const tinyxml2::XMLElement *dummyConfig = pt.RootElement()->FirstChildElement("dummyConfig");
  auto dummyConfig1 = get<std::string>(*dummyConfig, "dummyConfig1");
  auto dummyConfig4 = get<int>(pt, "dummyConfig.dummyConfig4");
  std::vector<T> dummyConfig3 = split<T>(get<std::string>(*dummyConfig, "dummyConfig3"), " ");

  print(dummyConfig1);
  print("dummyConfig4 =", dummyConfig4);
  print("dummyConfig3 =", dummyConfig3);
  return 0;
}
