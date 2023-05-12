#pragma once

#ifndef _CARIC_GAZEBO_PLUGIN_UTIL_H_
#define _CARIC_GAZEBO_PLUGIN_UTIL_H_

#include <vector>
// #include <cmath>
// #include <algorithm>
// #include <queue>
// #include <deque>
// #include <iostream>
// #include <fstream>
// #include <ctime>
// #include <cfloat>
// #include <iterator>
// #include <sstream>
// #include <string>
// #include <limits>
// #include <iomanip>
// #include <array>
// #include <thread>
// #include <random>
// #include <mutex>

#include <ros/ros.h>

// Printout colors
#define KNRM "\x1B[0m"
#define KRED "\x1B[31m"
#define KGRN "\x1B[32m"
#define KYEL "\x1B[33m"
#define KBLU "\x1B[34m"
#define KMAG "\x1B[35m"
#define KCYN "\x1B[36m"
#define KWHT "\x1B[37m"
#define RESET "\033[0m"

#define yolo() printf("Hello line: %s:%d. \n", __FILE__ , __LINE__);
#define yolos(...) printf("Hello line: %s:%d. ", __FILE__, __LINE__); printf(__VA_ARGS__); std::cout << std::endl;

using namespace std;

typedef Eigen::Vector3d Vector3d;
typedef Eigen::Quaterniond Quaternd;

namespace Util
{
    // Split the string into substrings
    vector<string> split(string s, string d)
    {
        vector<string> result;
        int pos = 0;
        while ((pos = s.find(d)) != string::npos)
        {
            string token = s.substr(0, pos);
            result.push_back(token);
            s.erase(0, pos + d.length());
        }
        result.push_back(s);

        return result;
    }
}

#endif //_CARIC_GAZEBO_PLUGIN_UTIL_H_