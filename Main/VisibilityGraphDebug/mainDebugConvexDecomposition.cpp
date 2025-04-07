#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
#include <Utils/Utils.h>
#include <Utils/Pragma.h>
#include <VisibilityGraph/Polygon.h>

using namespace RotationalVisibilityGraph;
using namespace Utils;
using T = double;
DECL_CGAL_CARTESIAN_TYPES_T
DECL_CGAL_POLYGON_TYPES_T

int main(){

    // std::vector<Vertex<T>> vertices = {
    //     Vertex<T>(-0.155417, 0.00537965),
    //     Vertex<T>(-0.0195918, 0.0558272),
    //     Vertex<T>(0.056904, 0.0559874),
    //     Vertex<T>(0.0863661, 0.0669301),
    //     Vertex<T>(0.109117, 0.00567555),
    //     Vertex<T>(0.0796549, -0.00526716),
    //     Vertex<T>(0.0216012, -0.0550808),
    //     Vertex<T>(-0.114224, -0.105528)
    // };

    std::vector<Vertex<T>> vertices = {
        Vertex<T>(0.0, 0.0),
        Vertex<T>(0.8660254037844387, 0.49999999999999994),
        Vertex<T>(0.4487991802004623, 0.8936326403234122),
        Vertex<T>(-0.11609291412523018, 0.993238357741943),
        Vertex<T>(-0.642787609686539, 0.7660444431189783),
        Vertex<T>(-0.9579895123154888, 0.2868032327110906),
        Vertex<T>(-0.957989512315489, -0.28680323271108993),
        Vertex<T>(-0.6427876096865395, -0.7660444431189779),
        Vertex<T>(-0.11609291412522998, -0.993238357741943),
        Vertex<T>(0.44879918020046206, -0.8936326403234123),
        Vertex<T>(0.8660254037844384, -0.5000000000000004)
    };

    // std::vector<Vertex<T>> vertices = {
    //     Vertex<T>(0, 0),
    //     Vertex<T>(1, 0),
    //     Vertex<T>(1, 1),
    //     Vertex<T>(0.8, 1.5),
    //     Vertex<T>(0.8, 2),
    //     Vertex<T>(0.2, 2),
    //     Vertex<T>(0.2, 1.5),
    //     Vertex<T>(0.0, 1)
    // };


    Polygon<T> polygon(vertices, false);
    polygon = polygon.getCounterClockWise();
    polygon.rotate(PI/2);
    polygon.draw("", "", true);
    auto convexDecomposition = polygon.convexDecomposition();
    print("Convex Decomposition: ", convexDecomposition.size());
    std::string pythonScript;
    PYTHON_IMPORTS(pythonScript) 
    for(const auto &poly: convexDecomposition){
        pythonScript += poly.draw("start", true);
    }
    pythonScript += "plt.show()\n";
    writeStringToFile<T>(pythonScript, "convexDecomposition.py");
    runPythonScriptAndRemove<T>("convexDecomposition.py");

    return 0;
}