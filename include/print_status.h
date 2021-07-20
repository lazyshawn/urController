#ifndef PRINT_STATUS_H
#define PRINT_STATUS_H

#include <stdio.h>
/* Reference: 
 * Colorful printf:
 *   https://www.cnblogs.com/lewki/p/14343894.html
 * template<typename... Args>:
 *   https://stackoverflow.com/questions/45891152
 */

template<typename... Args>
void printf_error(const char* f, Args... args) {
  // char printf_msg[100];
  // sprintf(printf_msg, f, args...);
  printf("\033[0m\033[1;31mERROR:\033[0m ");
  printf(f, args...);
}

template<typename... Args>
void printf_warning(const char* f, Args... args) {
  printf("\033[0m\033[1;33mWarning:\033[0m ");
  printf(f, args...);
}

template<typename... Args>
void printf_info(const char* f, Args... args) {
  printf("\033[0m\033[1;32mInfo:\033[0m ");
  printf(f, args...);
}

// int main() {
//   int a = 2;
//   printf_info("Expriment begin\n");
//   printf_error("%s\n", "test");
//   printf_warning("%s %d %d %p\n", "second test", 2, a, &a);
//   printf_info("%s\n", "test");
// }

#endif

