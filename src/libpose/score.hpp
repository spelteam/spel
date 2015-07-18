#ifndef _SCORE_HPP_
#define _SCORE_HPP_

// STL
#include <string>

namespace SPEL
{
  using namespace std;
  /// Used to evaluate accuracy of a detection
  class Score
  {
  public:
    Score(void);
    Score(float sc, string name, float _coeff = 1.0f);
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
    float getCoeff(void) const;
    void setCoeff(float _coeff);
    bool getIsWeak(void) const;
    void setIsWeak(bool _isWeak);

  private:
    /// detection score
    float score;
    /// detector name, name of the algorithm that generate the evaluation
    string detName;
    float coeff;
    /// signify label is from a badly localised part i.e. all very weak or all very similar detection scores
    bool isWeak;
  };

}

#endif  // _SCORE_HPP_

