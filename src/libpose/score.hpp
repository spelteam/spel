#ifndef _SCORE_HPP_
#define _SCORE_HPP_

#include <string>

using namespace std;

/// Used to evaluate accuracy of a detection
class Score
{
  public:
    Score(void);
    Score(float sc, string name);
/// copying all fields
    Score & operator=(const Score &s);
/// All this operators perform comparison by score values
    bool operator<(const Score &s) const;
    bool operator>(const Score &s) const;
    bool operator==(const Score &s) const;
/// This operator perform comparison by address
    bool operator!=(const Score &s) const;
/// All this functions just give access to the object fields
    float getScore(void) const;
    void setScore(float _score);
    string getDetName(void) const;
    void setDetName(string _detName);
  private:
/// detection score
    float score;
/// detector name, name of the algorithm that generate the evaluation
    string detName;
};

#endif  // _SCORE_HPP_
 
