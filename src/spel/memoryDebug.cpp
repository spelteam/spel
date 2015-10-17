#include <list>
#include <fstream>

#include "memoryDebug.hpp"

#ifdef MEMORY_DEBUG
#undef new

void OutputDebugString(const char *str)
{
  std::ofstream log;
  log.open("memorydebug.log", std::fstream::out | std::fstream::app);
  if (log.is_open())
  {
    log << str << std::endl;
    log.close();
  }
}

typedef struct {
  size_t address;
  size_t size;
} ALLOC_INFO;

typedef std::list<ALLOC_INFO*> AllocList;

AllocList *allocList;

void AddTrack(size_t addr, size_t asize)
{
  static int recursecount = 0;
  if (recursecount > 0)
    return;
  ++recursecount;
  ALLOC_INFO *info;

  if (!allocList) {
    allocList = new(AllocList);
  }

  info = new(ALLOC_INFO);
  info->address = addr;
  info->size = asize;
  allocList->insert(allocList->begin(), info);

  char buf[1024];
  snprintf(buf, 1024, "ADDRESS %zu\tSIZE %zu\n", addr, asize);
  OutputDebugString(buf);
  --recursecount;
}

void RemoveTrack(size_t addr)
{
  AllocList::iterator i;

  if (!allocList)
    return;
  for (i = allocList->begin(); i != allocList->end(); i++)
  {
    if ((*i)->address == addr)
    {
      char buf[1024];
      snprintf(buf, 1024, "FREED:\tADDRESS %zu\tSIZE %zu\n", (*i)->address, (*i)->size);
      OutputDebugString(buf);

      allocList->remove((*i));
      break;
    }
  }
}

void DumpUnfreed(void)
{
  AllocList::iterator i;
  size_t totalSize = 0;
  char buf[1024];

  if (!allocList)
    return;

  for (i = allocList->begin(); i != allocList->end(); i++)
  {
    snprintf(buf, 1024, "ADDRESS %zu\tSIZE %zu unfreed\n", (*i)->address, (*i)->size);
    OutputDebugString(buf);
    totalSize += (*i)->size;
  }
  snprintf(buf, 1024, "-----------------------------------------------------------\n");
  OutputDebugString(buf);
  snprintf(buf, 1024, "Total Unfreed: %zu bytes\n", totalSize);
  OutputDebugString(buf);
}

void * operator new(size_t size) noexcept
{
  void *ptr = (void *)malloc(size);
  AddTrack((size_t)ptr, size);
  return(ptr);
}

void * operator new[](size_t size) noexcept
{
  void *ptr = (void *)malloc(size);
  AddTrack((size_t)ptr, size);
  return(ptr);
}

void operator delete(void *p) noexcept
{
  RemoveTrack((size_t)p);
  free(p);
}

void operator delete[](void *p) noexcept
{
  RemoveTrack((size_t)p);
  free(p);
}

MemoryDebugAllocate::MemoryDebugAllocate(char const * filename, int lineNum)
{
  char buf[1024];
  snprintf(buf, 1024, "ALLOCATE:\t%s:%d\t", filename, lineNum);
  OutputDebugString(buf);
}

MemoryDebugAllocate::~MemoryDebugAllocate(void)
{
}

#endif // MEMORY_DEBUG

