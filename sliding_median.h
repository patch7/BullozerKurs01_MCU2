#ifndef __SLIDING
#define __SLIDING

#include <deque>
#include <algorithm>

template<typename Any, uint16_t size = 7>
class SlidingMedian
{
public:
  SlidingMedian() : fifo(size) {}
  SlidingMedian(const SlidingMedian&)            = delete;
  SlidingMedian(SlidingMedian&&)                 = delete;
  SlidingMedian& operator=(const SlidingMedian&) = delete;
  SlidingMedian& operator=(SlidingMedian&&)      = delete;
  inline void push(const Any&);
  const Any& get();
  ~SlidingMedian()                               = default;
private:
  std::deque<Any> fifo;
};

template<typename Any, uint16_t size> void SlidingMedian<Any, size>::push(const Any& a)
{
  fifo.pop_front();
  fifo.push_back(a);
}

template<typename Any, uint16_t size> const Any& SlidingMedian<Any, size>::get()
{
  std::sort(fifo.begin(), fifo.end());
  return *(fifo.begin() + fifo.size() / 2);
}

#endif /* __SLIDING */