#ifndef _SCORE_HPP_
#define _SCORE_HPP_

#include <string>

using namespace std;

class Score
{
  public:
    Score(void);
    Score(float sc, string name);
    Score & operator=(const Score &s);
    bool operator<(const Score &s) const;
    bool operator>(const Score &s) const;
    bool operator==(const Score &s) const;
    bool operator!=(const Score &s) const;
    float getScore(void) const;
    void setScore(float _score);
    string getDetName(void) const;
    void setDetName(string _detName);
  private:
    float score; //detection score
    string detName; //detector name
};

#endif  // _SCORE_HPP_
 
