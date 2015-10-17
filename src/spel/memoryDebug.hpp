#ifndef _MEMORYDEBUG_HPP_
#define _MEMORYDEBUG_HPP_

#ifdef MEMORY_DEBUG

#include <list>
#include <fstream>

typedef struct {
  size_t address;
  size_t size;
  char file[64];
  size_t line;
} ALLOC_INFO;

typedef std::list<ALLOC_INFO*> AllocList;

void OutputDebugString(const char *str);

void AddTrack(size_t addr, size_t asize, const char *fname, size_t lnum);

void RemoveTrack(size_t addr);

void DumpUnfreed(void);

void * operator new(size_t size, const char *file, int line);

void * operator new[](size_t size, const char *file, int line);

void operator delete(void *p);

void operator delete[](void *p);

#define DEBUG_NEW new(__FILE__, __LINE__)
#define new DEBUG_NEW

#define EIGEN_DONT_ALIGN

#endif // MEMORY_DEBUG

#endif  // _MEMORYDEBUG_HPP_
