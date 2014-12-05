// TODO: Macro for debugging, that can be disabled

#include <iostream>
#include <cstring>
#include "utils.h"
#include "ToolsForTesting/testing.h"

int main(int argc, char *argv[]) {
  if (argc > 1 && !strcmp(argv[1], "test")) {
    run_tests(argc - 2, argv + 2);
    return 0;
  }
  printf("%d\n", argc);
  return 0;
}