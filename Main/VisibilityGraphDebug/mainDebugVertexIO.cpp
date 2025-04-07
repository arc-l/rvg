#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Polygon.h>
#include <Utils/Utils.h>
#include <Utils/Timer.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/Utils.h>


using namespace RotationalVisibilityGraph;
using namespace Utils;
using T = double;

int main(int argc, char* argv[]){
  print("Hello World!");
  tinyxml2::XMLDocument pt;
  pt.LoadFile(argv[1]);
  const tinyxml2::XMLElement *vertexRoot= pt.RootElement()->FirstChildElement("Vertex");
  while(vertexRoot){
    auto vertex = getVertex<T>(*vertexRoot);
    print(*vertex);
    vertexRoot = vertexRoot->NextSiblingElement("Vertex");
  }
  return 0;
}