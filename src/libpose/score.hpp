#ifndef _SCORE_HPP_
#define _SCORE_HPP_

#include <string>

using namespace std;

class Score
{
  public:
//TODO (Vitaliy Koshura): Need implementation
    Score(void);
    Score(float sc, string name);
    Score & operator=(const Score &s);
//TODO (Vitaliy Koshura): Need implementation
    bool operator<(const Score &s) const;
//TODO (Vitaliy Koshura): Need implementation
    bool operator>(const Score &s) const;
//TODO (Vitaliy Koshura): Need implementation
    bool operator==(const Score &s) const;
//TODO (Vitaliy Koshura): Need implementation
    bool operator!=(const Score &s) const;
//TODO (Vitaliy Koshura): Need implementation
    float val(void);
  float getScore(void) const;
  string getDetName(void) const;
  private:
    float score; //detection score
    string detName; //detector name
};

#endif  // _SCORE_HPP_
 
