#ifndef _SCORE_HPP_
#define _SCORE_HPP_

#include <string>

using namespace std;

class Score
{
  public:
    Score();
    Score(float sc, string name);
    Score & operator=(const Score &s);
    bool operator<(const Score &s) const;
    bool operator>(const Score &s) const;
    bool operator==(const Score &s) const;
    bool operator!=(const Score &s) const;
  private:
    float score; //detection score
    string detName; //detector name
};

#endif  // _SCORE_HPP_
 
