#include "Utils.h"

namespace RotationalVisibilityGraph::Utils {
template<>
std::vector<std::string> split<std::string>(const std::string &l, const std::string &sep) {
  int i, last = -1;
  std::vector<std::string> ret;
  for (i = 0; i < (int) l.size(); i++)
    if (std::find(sep.begin(), sep.end(), l[i]) != sep.end()) {
      if (last < i - 1)
        ret.push_back(l.substr(last + 1, i - last - 1));
      last = i;
    }
  if (last < i - 1)
    ret.push_back(l.substr(last + 1, i - last - 1));
  return ret;
}
bool beginsWith(const std::string &l, const std::string &s) {
  return l.length() >= s.length() && l.substr(0, s.length()) == s;
}
bool endsWith(const std::string &l, const std::string &s) {
  return l.length() >= s.length() && l.substr(l.size() - s.length(), s.length()) == s;
}
std::string toUpper(const std::string &l) {
  std::string ret = l;
  std::for_each(ret.begin(), ret.end(), [&](char c) {
    return std::toupper(c);
  });
  return ret;
}

//put/get
const tinyxml2::XMLElement *getChild(const tinyxml2::XMLElement &pt, const std::string &name) {
  for (const tinyxml2::XMLElement *v = pt.FirstChildElement(); v; v = v->NextSiblingElement())
    if (v->Name() == name)
      return v;
  return NULL;
}
tinyxml2::XMLElement *getChild(tinyxml2::XMLElement &pt, const std::string &name) {
  for (tinyxml2::XMLElement *v = pt.FirstChildElement(); v; v = v->NextSiblingElement())
    if (v->Name() == name)
      return v;
  return NULL;
}
tinyxml2::XMLElement *addChild(tinyxml2::XMLElement &pt, const std::string &name) {
  //for(tinyxml2::XMLElement* v=pt.FirstChildElement(); v; v=v->NextSiblingElement())
  //  if(v->Name()==name)
  //    return v;
  tinyxml2::XMLElement *node = pt.GetDocument()->NewElement(name.c_str());
  pt.InsertEndChild(node);
  return node;
}
const tinyxml2::XMLElement *getAttributeInfo(const tinyxml2::XMLElement &pt, std::string &name) {
  const tinyxml2::XMLElement *curr = &pt;
  std::vector<std::string> paths = split<std::string>(name, ".");
  for (int i = 0; i < (int) paths.size(); i++)
    if (paths[i] == "<xmlattr>") {
      ASSERT(i == (int) paths.size() - 2)
      name = paths[i + 1];
      return curr;
    }
    else {
      curr = getChild(*curr, paths[i]);
      if (!curr)
        return curr;
    }
  name = "";
  return curr;
}
tinyxml2::XMLElement *getAttributeInfoPut(tinyxml2::XMLElement &pt, std::string &name) {
  tinyxml2::XMLElement *curr = &pt;
  std::vector<std::string> paths = split<std::string>(name, ".");
  for (int i = 0; i < (int) paths.size(); i++)
    if (paths[i] == "<xmlattr>") {
      ASSERT(i == (int) paths.size() - 2)
      name = paths[i + 1];
      return curr;
    }
    else {
      tinyxml2::XMLElement *tmp = getChild(*curr, paths[i]);
      if (!tmp) {
        tinyxml2::XMLElement *node = pt.GetDocument()->NewElement(paths[i].c_str());
        curr->InsertEndChild(node);
        curr = node;
      }
      else {
        curr = tmp;
      }
    }
  name = "";
  return curr;
}
bool hasAttribute(const tinyxml2::XMLElement &pt, const std::string &name) {
  std::string nameProcessed = name;
  const tinyxml2::XMLElement *e = getAttributeInfo(pt, nameProcessed);
  if (!e)
    return false;
  else if (nameProcessed.empty())
    return true;
  else return e->Attribute(nameProcessed.c_str()) != NULL;
}
std::vector<std::string> getNames(std::string val, char sep) {
  std::vector<std::string> names;
  while (true) {
    size_t pos = val.find_first_of(sep);
    if (pos == std::string::npos)
      break;
    names.push_back(val.substr(0, pos));
    if (pos == val.length() - 1) {
      val = "";
      break;
    }
    else val = val.substr(pos + 1);
  }
  if (!val.empty())
    names.push_back(val);
  return names;
}
std::vector<std::string> getNames(const tinyxml2::XMLDocument &pt, const std::string &name, char sep) {
  std::string str = get<std::string>(pt, name, "");
  return getNames(str, sep);
}

//parsePtree
std::vector<std::string> toParams(int argc, char **argv) {
  std::vector<std::string> params;
  for (int i = 0; i < argc; i++)
    params.push_back(argv[i]);
  return params;
}
std::string parseProps(int argc, char **argv, tinyxml2::XMLElement &pt) {
  return parseProps(toParams(argc, argv), pt);
}
std::string parseProps(const std::vector<std::string> &params, tinyxml2::XMLElement &pt) {
  std::string addParam;
  for (int i = 0; i < (int) params.size(); i++) {
    const std::string &str = params[i];
    size_t pos = str.find("=");
    if (pos != std::string::npos) {
      std::string LHS = str.substr(0, pos);
      std::string RHS = str.substr(pos + 1);
      put<std::string>(pt, LHS.c_str(), RHS.c_str());
      addParam += "_" + str;
    }
  }
  return addParam + "_";
}
std::string parseProps(int argc, char **argv, tinyxml2::XMLDocument &pt) {
  return parseProps(argc, argv, *(pt.RootElement()));
}
std::string parseProps(const std::vector<std::string> &params, tinyxml2::XMLDocument &pt) {
  return parseProps(params, *(pt.RootElement()));
}

}