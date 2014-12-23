#ifndef _SCORE_HPP_
#define _SCORE_HPP_

#include <string>

using namespace std;

// Used to evaluate accuracy of a detection
class Score
{
  public:
    Score(void);
    Score(float sc, string name);
	Score & operator=(const Score &s); // copying all fields	
	// All this operators perform comparison by score values
    bool operator<(const Score &s) const;
    bool operator>(const Score &s) const;
    bool operator==(const Score &s) const;
	// This operator perform comparison by address
    bool operator!=(const Score &s) const;
	// All this functions just give access to the object fields
    float getScore(void) const;
    void setScore(float _score);
    string getDetName(void) const;
    void setDetName(string _detName);
  private:
    float score; // detection score
    string detName; // detector name, name of the algorithm that generate the evaluation
};

#endif  // _SCORE_HPP_
 
