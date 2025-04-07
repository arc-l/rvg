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
  const tinyxml2::XMLElement *polygonRoot= pt.RootElement()->FirstChildElement("Polygon");
  int cnt=0;
  while(polygonRoot){
    Polygon<T> polygon = getPolygon<T>(*polygonRoot);
    print("Polygon", cnt);
    for (const auto &vertex : polygon.getVertices()) {
      print(vertex);
    }
    polygonRoot = polygonRoot->NextSiblingElement("Polygon");
    cnt++;
  }
  return 0;
}
