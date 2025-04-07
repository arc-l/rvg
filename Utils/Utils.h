#ifndef ROTATIONALVISIBILITYGRAPH_UTILS_UTILS_H_
#define ROTATIONALVISIBILITYGRAPH_UTILS_UTILS_H_

#include <iostream>
#include <iomanip>
#include <vector>
#include <unordered_map>
#include <Utils/Pragma.h>
#include "tinyxml2.h"
#include <filesystem>
#include <tinyxml2.h>
#include <string>
#include <algorithm>
#include <fstream>
#include <omp.h>

namespace RotationalVisibilityGraph::Utils {

template<typename T>
bool writeVectorToFile(const std::vector<T>& vec, const std::string& filePath) {
  std::ofstream outFile(filePath);  // Create and open the output file stream

  // Check if the file is successfully opened
  if (!outFile.is_open()) {
    std::cerr << "Failed to open the file at " << filePath << std::endl;
    return false;  // Return false if the file cannot be opened
  }

  // Write each element of the vector to the file
  for (const T& item : vec) {
    outFile << item << std::endl;
  }

  outFile.close();  // Close the file stream
  return true;  // Return true if the operation is successful
}

template<typename T>
bool writeStringToFile(const std::string& str, const std::string& filePath) {
  std::ofstream outFile(filePath);  // Create and open the output file stream

  // Check if the file is successfully opened
  if (!outFile.is_open()) {
    std::cerr << "Failed to open the file at " << filePath << std::endl;
    return false;  // Return false if the file cannot be opened
  }

  outFile << str;  // Write the string to the file
  outFile.close();  // Close the file stream
  return true;  // Return true if the operation is successful
}

template<typename T>
bool createTempFolderIfNotExists(const std::string& folderPath) {
  // Check if the folder already exists
  if (std::filesystem::exists(folderPath)) {
    return true;  // Return true if the folder already exists
  }
  // Create the folder if it does not exist
  if (std::filesystem::create_directory(folderPath)) {
    return true;  // Return true if the folder is successfully created
  } else {
    std::cerr << "Failed to create the folder at " << folderPath << std::endl;
    return false;  // Return false if the folder cannot be created
  }
}

template<typename T>
std::string createTempFolder(){
  std::string folderPath = std::filesystem::temp_directory_path().string() + "/rotational-visibility-graph";
  if(createTempFolderIfNotExists<T>(folderPath)){
    return folderPath;
  }
  else throw std::runtime_error("Failed to create temporary folder");
}

template<typename T>
bool runPythonScriptAndRemove(const std::string& scriptPath) {
  // Run the Python script
#ifdef PYTHON_EXECUTABLE
  std::string command = std::string(PYTHON_EXECUTABLE) + " " + scriptPath;
#else
  std::string command = "python3 " + scriptPath;
#endif
  int status = system(command.c_str());

  // Check if the Python script is successfully executed
  if (status != 0) {
    std::cerr << "Failed to run the Python script at " << scriptPath << std::endl;
    return false;  // Return false if the Python script cannot be executed
  }

  // Remove the Python script
  if (std::filesystem::remove(scriptPath)) {
    return true;  // Return true if the Python script is successfully executed and removed
  } else {
    std::cerr << "Failed to remove the Python script at " << scriptPath << std::endl;
    return false;  // Return false if the Python script cannot be removed
  }
}


extern bool beginsWith(const std::string &l, const std::string &s);
extern bool endsWith(const std::string &l, const std::string &s);
extern std::string toUpper(const std::string &l);

template<typename T>
std::vector<T> split(const std::string &l, const std::string &sep) {
  std::vector<std::string> tokens = split<std::string>(l, sep);
  std::vector<T> ret;
  for (const auto &token : tokens) {
    ret.push_back(std::stod(token));
  }
  return ret;
}

template<>
std::vector<std::string> split<std::string>(const std::string &l, const std::string &sep);

template<typename T>
void setPrecision(int precision) {
  std::cout << std::fixed << std::setprecision(precision);
}

template<typename T>
struct is_std_vector : std::false_type {};

template<typename T, typename Alloc>
struct is_std_vector<std::vector<T, Alloc>> : std::true_type {};
inline void print() {
  std::cout << std::endl;
}
template<typename T>
inline void print(const std::vector<T> &num, char end = '\n') {
  for (const auto &i : num) {
    std::cout << i << " ";
  }
  if (end == '\n')
    std::cout << std::endl;
  else
    std::cout << end;
}

template<typename T, typename ...TAIL>
inline void print(const T &t, TAIL... tail) {
  if constexpr (is_std_vector<std::decay_t<T>>::value)
    print(t, ' ');
  else
    std::cout << t << ' ';
  print(tail...);
}

inline void hash_combine(std::size_t &seed, size_t hash) {
  seed ^= hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

//property tree
template<typename SCALAR>
struct PtreeSafeType {
  typedef int SAFE_SCALAR;
  static void put(tinyxml2::XMLElement &pt, const std::string &name, const SCALAR &val) {
    SAFE_SCALAR def = val;
    if (name.empty()) {
      std::string text = std::to_string(def);
      pt.SetText(text.c_str());
    }
    else pt.SetAttribute(name.c_str(), def);
  }
  static int get(const tinyxml2::XMLElement &pt, const std::string &name, const SCALAR &val) {
    SAFE_SCALAR def = val;
    if (name.empty())
      pt.QueryIntText(&def);
    else pt.QueryIntAttribute(name.c_str(), &def);
    return def;
  }
  static int get(const tinyxml2::XMLElement &pt, const std::string &name) {
    SAFE_SCALAR def = 0;
    if (name.empty()) {
      ASSERT(pt.QueryIntText(&def) == tinyxml2::XML_SUCCESS)
    }
    else {
      ASSERT(pt.QueryIntAttribute(name.c_str(), &def) == tinyxml2::XML_SUCCESS)
    }
    return def;
  }
  static SAFE_SCALAR toSafe(SCALAR val) {
    return val;
  }
};

template<>
struct PtreeSafeType<float> {
  typedef float SAFE_SCALAR;
  //text
  static void put(tinyxml2::XMLElement &pt, const std::string &name, const float &val) {
    SAFE_SCALAR def = val;
    if (name.empty()) {
      std::string text = std::to_string(def);
      pt.SetText(text.c_str());
    }
    else pt.SetAttribute(name.c_str(), def);
  }
  static float get(const tinyxml2::XMLElement &pt, const std::string &name, const float &val) {
    SAFE_SCALAR def = val;
    if (name.empty())
      pt.QueryFloatText(&def);
    else pt.QueryFloatAttribute(name.c_str(), &def);
    return def;
  }
  static float get(const tinyxml2::XMLElement &pt, const std::string &name) {
    SAFE_SCALAR def = 0;
    if (name.empty()) {
      ASSERT(pt.QueryFloatText(&def) == tinyxml2::XML_SUCCESS)
    }
    else {
      ASSERT(pt.QueryFloatAttribute(name.c_str(), &def) == tinyxml2::XML_SUCCESS)
    }
    return def;
  }
  static SAFE_SCALAR toSafe(float val) {
    return val;
  }
};
template<>
struct PtreeSafeType<double> {
  typedef float SAFE_SCALAR;
  static void put(tinyxml2::XMLElement &pt, const std::string &name, const double &val) {
    SAFE_SCALAR def = toSafe(val);
    if (name.empty()) {
      std::string text = std::to_string(def);
      pt.SetText(text.c_str());
    }
    else pt.SetAttribute(name.c_str(), def);
  }
  static double get(const tinyxml2::XMLElement &pt, const std::string &name, const double &val) {
    SAFE_SCALAR def = toSafe(val);
    if (name.empty())
      pt.QueryFloatText(&def);
    else pt.QueryFloatAttribute(name.c_str(), &def);
    return def;
  }
  static double get(const tinyxml2::XMLElement &pt, const std::string &name) {
    SAFE_SCALAR def = 0;
    if (name.empty()) {
      ASSERT(pt.QueryFloatText(&def) == tinyxml2::XML_SUCCESS)
    }
    else {
      ASSERT(pt.QueryFloatAttribute(name.c_str(), &def) == tinyxml2::XML_SUCCESS)
    }
    return def;
  }
  static SAFE_SCALAR toSafe(double val) {
    return (SAFE_SCALAR) val;
  }
};

template<>
struct PtreeSafeType<std::string> {
  typedef std::string SAFE_SCALAR;
  static void put(tinyxml2::XMLElement &pt, const std::string &name, const std::string &val) {
    if (name.empty()) {
      pt.SetText(val.c_str());
    }
    else pt.SetAttribute(name.c_str(), val.c_str());
  }
  static std::string get(const tinyxml2::XMLElement &pt, const std::string &name, const std::string &val) {
    if (name.empty())
      return pt.GetText();
    else {
      const char *ret = pt.Attribute(name.c_str());
      if (!ret)
        return val;
      else return ret;
    }
  }
  static std::string get(const tinyxml2::XMLElement &pt, const std::string &name) {
    const char *ret = NULL;
    if (name.empty())
      ret = pt.GetText();
    else ret = pt.Attribute(name.c_str());
    ASSERT(ret != 0)
    return ret;
  }
  static SAFE_SCALAR toSafe(std::string val) {
    return val;
  }
};

//put/get
const tinyxml2::XMLElement *getChild(const tinyxml2::XMLElement &pt, const std::string &name);
tinyxml2::XMLElement *getChild(tinyxml2::XMLElement &pt, const std::string &name);
tinyxml2::XMLElement *addChild(tinyxml2::XMLElement &pt, const std::string &name);
const tinyxml2::XMLElement *getAttributeInfo(const tinyxml2::XMLElement &pt, std::string &name);
tinyxml2::XMLElement *getAttributeInfoPut(tinyxml2::XMLElement &pt, std::string &name);
bool hasAttribute(const tinyxml2::XMLElement &pt, const std::string &name);
template<typename T>
void put(tinyxml2::XMLElement &pt, const std::string &name, const T &val) {
  std::string nameProcessed = name;
  tinyxml2::XMLElement *e = getAttributeInfoPut(pt, nameProcessed);
  ASSERT(e)
  PtreeSafeType<T>::put(*e, nameProcessed, val);
}
template<typename T>
void put(tinyxml2::XMLDocument &pt, const std::string &name, const T &val) {
  put<T>(*(pt.RootElement()), name, val);
}
template<typename T>
void putCond(tinyxml2::XMLElement &pt, const std::string &path, T val) {
  if (!hasAttribute(pt, path))
    put<T>(pt, path, val);
}
template<typename T>
void putCond(tinyxml2::XMLDocument &pt, const std::string &path, T val) {
  putCond<T>(*(pt.RootElement()), path, val);
}
template<typename T>
T get(const tinyxml2::XMLElement &pt, const std::string &name, const T &val) {
  std::string nameProcessed = name;
  const tinyxml2::XMLElement *e = getAttributeInfo(pt, nameProcessed);
  if (!e)
    return val;
  else return PtreeSafeType<T>::get(*e, nameProcessed, val);
}
template<typename T>
T get(const tinyxml2::XMLDocument &pt, const std::string &name, const T &val) {
  return get<T>(*(pt.RootElement()), name, val);
}
template<typename T>
T get(const tinyxml2::XMLElement &pt, const std::string &name) {
  std::string nameProcessed = name;
  const tinyxml2::XMLElement *e = getAttributeInfo(pt, nameProcessed);
  if (!e)
    throw std::runtime_error("Attribute " + name + " does not exist!");
  return PtreeSafeType<T>::get(*e, nameProcessed);
}
template<typename T>
T get(const tinyxml2::XMLDocument &pt, const std::string &name) {
  return get<T>(*(pt.RootElement()), name);
}
template<typename T>
std::vector<T> getVector(const tinyxml2::XMLElement &pt, const std::string &name, const std::string& sep = " ", const std::vector<T> &val = {}) {
  std::string nameProcessed = name;
  const tinyxml2::XMLElement *e = getAttributeInfo(pt, nameProcessed);
  if (!e)
    return val;
  return split<T>(get<std::string>(pt, name), sep);
}
extern std::vector<std::string> getNames(std::string val, char sep = ',');
extern std::vector<std::string> getNames(const tinyxml2::XMLDocument &pt, const std::string &name, char sep = ',');

//parsePtree
std::vector<std::string> toParams(int argc, char **argv);
std::string parseProps(int argc, char **argv, tinyxml2::XMLElement &pt);
std::string parseProps(const std::vector<std::string> &params, tinyxml2::XMLElement &pt);
std::string parseProps(int argc, char **argv, tinyxml2::XMLDocument &pt);
std::string parseProps(const std::vector<std::string> &params, tinyxml2::XMLDocument &pt);

}

#endif //ROTATIONALVISIBILITYGRAPH_UTILS_UTILS_H_
