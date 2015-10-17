#ifndef _MEMORYDEBUG_HPP_
#define _MEMORYDEBUG_HPP_

#ifdef MEMORY_DEBUG

void OutputDebugString(const char *str);

class MemoryDebugAllocate
{
public:
  MemoryDebugAllocate(char const *filename, int lineNum);
  ~MemoryDebugAllocate(void);
};

template <class T> inline T *operator*(const MemoryDebugAllocate &stamp, T *p)
{
  return p;
}

void AddTrack(size_t addr, size_t asize);

void RemoveTrack(size_t addr);

#define MEMORY_DEBUG_NEW MemoryDebugAllocate(__FILE__, __LINE__) * new
#define new MEMORY_DEBUG_NEW

#define EIGEN_DONT_ALIGN

#endif // MEMORY_DEBUG

#endif  // _MEMORYDEBUG_HPP_
