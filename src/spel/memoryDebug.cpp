#include <list>
#include <fstream>

#ifdef MEMORY_DEBUG

typedef struct {
  size_t address;
  size_t size;
  char file[64];
  size_t line;
} ALLOC_INFO;

typedef std::list<ALLOC_INFO*> AllocList;

AllocList *allocList;

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

void AddTrack(size_t addr, size_t asize, const char *fname, size_t lnum)
{
  ALLOC_INFO *info;

  if (!allocList) {
    allocList = new(AllocList);
  }

  info = new(ALLOC_INFO);
  info->address = addr;
  strncpy(info->file, fname, 63);
  info->line = lnum;
  info->size = asize;
  allocList->insert(allocList->begin(), info);

  char buf[1024];
  sprintf(buf, "ALLOCATED:\t%s:%d\tADDRESS %d\tSIZE %d\n", fname, lnum, addr, asize);
  OutputDebugString(buf);
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
      sprintf(buf, "FREED:\t%s:%d\tADDRESS %d\tSIZE %d\n", (*i)->file, (*i)->line, (*i)->address, (*i)->size);
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
    sprintf(buf, "%s:%d\tADDRESS %d\tSIZE %d unfreed\n", (*i)->file, (*i)->line, (*i)->address, (*i)->size);
    OutputDebugString(buf);
    totalSize += (*i)->size;
  }
  sprintf(buf, "-----------------------------------------------------------\n");
  OutputDebugString(buf);
  sprintf(buf, "Total Unfreed: %d bytes\n", totalSize);
  OutputDebugString(buf);
}

void * operator new(size_t size, const char *file, int line)
{
  void *ptr = (void *)malloc(size);
  AddTrack((size_t)ptr, size, file, line);
  return(ptr);
}

void * operator new[](size_t size, const char *file, int line)
{
  void *ptr = (void *)malloc(size);
  AddTrack((size_t)ptr, size, file, line);
  return(ptr);
}

void operator delete(void *p)
{
  RemoveTrack((size_t)p);
  free(p);
}

void operator delete[](void *p)
{
  RemoveTrack((size_t)p);
  free(p);
}

#endif // MEMORY_DEBUG
