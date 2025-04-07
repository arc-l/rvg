#include <Utils/Utils.h>
#include <Utils/Timer.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/Vertex.h>
#include <VisibilityGraph/Polygon.h>
#include <VisibilityGraph/Utils.h>

using namespace RotationalVisibilityGraph;
using namespace Utils;
typedef double T;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T
int main(int argc, char * argv[]){
  tinyxml2::XMLDocument pt;
  pt.LoadFile(argv[1]);

  Polygon<T> robot = getRobot<T>(pt);
  Polygon<T> inscribedCircle;

  Timer timer("Find Maximum Inscribed Circle", true);
  for(int i = 0; i < 1000; i++) 
    inscribedCircle = robot.findMaximumInscribedCircle(36);
  timer.stop();

  std::string pythonScript;
  PYTHON_IMPORTS(pythonScript)
  pythonScript += robot.draw("obs", false);
  pythonScript += inscribedCircle.draw("start", false);
  pythonScript += "plt.axis('equal')\n";
  pythonScript += "plt.show()\n";
  std::string path = "draw.py";
  writeStringToFile<T>(pythonScript, path);
  runPythonScriptAndRemove<T>(path);
  return 0;
}