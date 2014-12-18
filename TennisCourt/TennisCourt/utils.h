#ifndef UTILS_H__
#define UTILS_H__

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <map>

// Debug prints stuff only when not in release mode
//
#ifdef NDEBUG
#define DEBUG(x) do {} while (0)
#else
#define DEBUG(x) do { std::cerr << x << std::endl; } while (0)
#endif

// Info always prints stuff to stderr
#define INFO(x) do { std::cerr << x << std::endl; } while (0)

extern std::map<std::string, std::vector<std::string>> g_args;
void ParseArguments(int argc, char *argv[]);

#endif